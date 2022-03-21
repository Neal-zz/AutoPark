#include "MotionController.h"
#include "ComCaspTest.h"
#include "GxCamTest.h"
#include "Detector.h"
#include "config.h"

#include <Eigen\Dense>
#include <iostream>
#include <fstream>
#include <conio.h>
#include <opencv2/opencv.hpp>
#include <chrono>   
#include <string>

/*timer*/
auto tic = []()
{
	return std::chrono::system_clock::now();
};

auto toc = [](std::chrono::system_clock::time_point start, const std::string& name)
{
	std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
	std::chrono::duration<double, std::milli> duration_ms = end - start;
	std::cout << name << '\t' << duration_ms.count() << "ms" << std::endl;

	return;
};

// input cartVel, get the motion trace.
void forwardControl()
{
	CartPara cartTest;
	cartTest.wheelsRadius = 0.15;
	cartTest.wheelsDistance = 0.4;
	cartTest.linVelLim = 1.2;
	cartTest.linAccLim = 0.6; 
	cartTest.angVelLim = 1.5;
	cartTest.angAccLim = 0.8;
	cartTest.communFreq = 10;

	MotionController mC(cartTest);
	mC.updateCartIniPos(Eigen::Vector3d::Zero());
	Eigen::Vector2d cartVel(-0.5, 0.05);
	mC.updateCartCurrVel(cartVel);

	std::ofstream outfile;
	outfile.open("fCData.txt", std::ios::trunc | std::ios::out);

	int maxSimTime = 20;
	for (int i = 0; i <= cartTest.communFreq*maxSimTime; i++)
	{
		Eigen::Vector2d wheelsVeli = mC.fromCartVel2WheelsVel2();
		mC.updateWheelsVel(wheelsVeli);
		Eigen::Vector3d cartAcci = mC.fromWheelsVel2CartVel3();
		Eigen::Vector3d cartPosNi;
		Eigen::Vector3d cartPosi = mC.getCartCurrPos();
		cartPosNi << cartAcci(0) / cartTest.communFreq + cartPosi(0),
			cartAcci(1) / cartTest.communFreq + cartPosi(1),
			cartAcci(2) / cartTest.communFreq + cartPosi(2);
		mC.updateCartCurrPos(cartPosNi);

		outfile << cartPosi(0) << '\t' << cartPosi(1) << '\t' << cartPosi(2) << '\n';
	}
	outfile.close();
}

// from initial position to the target point.
void closedLoopControl()
{
	CartPara cartTest;
	cartTest.wheelsRadius = 0.15;
	cartTest.wheelsDistance = 0.4;
	cartTest.linVelLim = 1.2;
	cartTest.linAccLim = 0.6;
	cartTest.angVelLim = 1.5;
	cartTest.angAccLim = 0.8;
	cartTest.communFreq = 10;

	MotionController mC(cartTest);
	Eigen::Vector3d cartIniPos(1, 0, 0);
	mC.updateCartIniPos(cartIniPos);
	Eigen::Vector3d tarPos(5, 4, 0);
	mC.updateTarPos(tarPos);

	std::ofstream outfile; // log the "closed loop control test data".
	outfile.open("cLCData.txt", std::ios::trunc | std::ios::out);

	double disThresh = 0.05;
	int maxSimTime = 100;
	for (int i = 0; i <= cartTest.communFreq*maxSimTime; i++)
	{
		Eigen::Vector3d cartPosi = mC.getCartCurrPos();
		double disErr = sqrt(pow(cartPosi(0) - tarPos(0), 2) + pow(cartPosi(1) - tarPos(1), 2));
		if (disErr < disThresh) { break; }; // if it's close enough, break.
		double angErr = atan2(tarPos(1) - cartPosi(1), tarPos(0) - cartPosi(0)) - cartPosi(2);
		if (angErr > EIGEN_PI)
			angErr -= 2 * EIGEN_PI;
		else if (angErr < -EIGEN_PI)
			angErr += 2 * EIGEN_PI;
		
		// compute next position and orientation from information at time i.
		Eigen::Vector2d cartVeli = mC.getCartCurrVel();
		Eigen::Vector2d wheelsVeli = mC.fromCartVel2WheelsVel2();
		mC.updateWheelsVel(wheelsVeli);
		Eigen::Vector3d cartAcci = mC.fromWheelsVel2CartVel3();
		Eigen::Vector3d cartPosNi;
		cartPosNi << cartAcci(0) / cartTest.communFreq + cartPosi(0),
			cartAcci(1) / cartTest.communFreq + cartPosi(1),
			cartAcci(2) / cartTest.communFreq + cartPosi(2);
		mC.updateCartCurrPos(cartPosNi);

		// compute next velocity.
		Eigen::Vector2d cartVelNi(std::min(disErr, cartTest.linVelLim), fabs(angErr)>cartTest.angVelLim ? cartTest.angVelLim*angErr / fabs(angErr) : angErr);
		if ((cartVelNi(0) - cartVeli(0)) * cartTest.communFreq > cartTest.linAccLim)
			cartVelNi(0) = cartVeli(0) + cartTest.linAccLim / cartTest.communFreq;
		else if ((cartVelNi(0) - cartVeli(0)) * cartTest.communFreq < -cartTest.linAccLim)
			cartVelNi(0) = cartVeli(0) - cartTest.linAccLim / cartTest.communFreq;
		if ((cartVelNi(1) - cartVeli(1)) * cartTest.communFreq > cartTest.angAccLim)
			cartVelNi(1) = cartVeli(1) + cartTest.angAccLim / cartTest.communFreq;
		else if ((cartVelNi(1) - cartVeli(1)) * cartTest.communFreq < -cartTest.angAccLim)
			cartVelNi(1) = cartVeli(1) - cartTest.angAccLim / cartTest.communFreq;
		mC.updateCartCurrVel(cartVelNi);
		wheelsVeli = mC.fromCartVel2WheelsVel2();
		mC.updateWheelsVel(wheelsVeli);

		outfile << cartPosi(0) << '\t' << cartPosi(1) << '\t' << cartPosi(2)
			<< '\t' << cartVeli(0) << '\t' << cartVeli(1) <<'\n';
	}
	outfile.close();
}

MatrixPath pathGenerator(const Eigen::Vector3d& initPose, const Eigen::Vector3d& finalPose)
{
	Eigen::Matrix<double, 2, 3> parabolic(Eigen::Matrix<double, 2, 3>::Zero()); // order: p, h, k
	parabolic(0, 0) = initPose(1) * initPose(1) / 4 / (initPose(0) - initPose(1) / tan(finalPose(2)));
	parabolic(0, 1) = -parabolic(0, 0) / tan(finalPose(2)) / tan(finalPose(2));
	parabolic(0, 2) = -2 * parabolic(0, 0) / tan(finalPose(2));
	parabolic(1, 0) = initPose(0) * initPose(0) / 4 / (initPose(1) - initPose(0)*tan(finalPose(2)));
	parabolic(1, 1) = -2 * parabolic(1, 0)*tan(finalPose(2));
	parabolic(1, 2) = -parabolic(1, 0)*tan(finalPose(2))*tan(finalPose(2));

	const int angleNum=13;
	Eigen::VectorXd initOri(angleNum);
	initOri << initPose(2) - 60 * EIGEN_PI / 180, initPose(2) - 50 * EIGEN_PI / 180, initPose(2) - 40 * EIGEN_PI / 180,
		initPose(2) - 30 * EIGEN_PI / 180, initPose(2) - 20 * EIGEN_PI / 180, initPose(2) - 10 * EIGEN_PI / 180,
		initPose(2) - 0 * EIGEN_PI / 180, initPose(2) + 10 * EIGEN_PI / 180, initPose(2) + 20 * EIGEN_PI / 180,
		initPose(2) + 30 * EIGEN_PI / 180, initPose(2) + 40 * EIGEN_PI / 180, initPose(2) + 50 * EIGEN_PI / 180,
		initPose(2) + 60 * EIGEN_PI / 180;
	Eigen::MatrixXd cubic1(angleNum, 4);
	Eigen::MatrixXd cubic2(angleNum, 4);
	Eigen::VectorXi cubic1State(angleNum); // 0: good; 1: sigular; 2:wrong direction.
	Eigen::VectorXi cubic2State(angleNum);
	cubic1State = Eigen::Matrix<int, angleNum, 1>::Zero();
	cubic2State = Eigen::Matrix<int, angleNum, 1>::Zero();
	for (int i = 0; i < angleNum; i++)
	{
		if (fabs(initOri(i) - EIGEN_PI / 2) < 0.001 || fabs(initOri(i) + EIGEN_PI / 2) < 0.001 || fabs(initPose(0)) < 0.001) // other criterions are discussed later.
			cubic1State(i) = 1;
		cubic1(i, 0) = 0;
		cubic1(i, 1) = tan(finalPose(2));
		cubic1(i, 2) = (3 * initPose(1) - initPose(0)*(tan(initOri(i)) + 2 * tan(finalPose(2)))) / initPose(0) / initPose(0);
		cubic1(i, 3) = (initPose(0)*(tan(initOri(i)) + tan(finalPose(2))) - 2 * initPose(1)) / initPose(0) / initPose(0) / initPose(0);
		if (fabs(initOri(i)) < 0.001 || fabs(fabs(initOri(i)) - EIGEN_PI) < 0.001 || fabs(initPose(1)) < 0.001)
			cubic2State(i) = 1;
		cubic2(i, 0) = 0;
		cubic2(i, 1) = 1 / tan(finalPose(2));
		cubic2(i, 2) = (3 * initPose(0) - initPose(1)*(1 / tan(initOri(i)) + 2 / tan(finalPose(2)))) / initPose(1) / initPose(1);
		cubic2(i, 3) = (initPose(1)*(1 / tan(initOri(i)) + 1 / tan(finalPose(2))) - 2 * initPose(0)) / initPose(1) / initPose(1) / initPose(1);
	}

	MatrixPath pathOut;
	int pathId = 12;

	for (int i = 0; i < 20; i++)
	{
		double z2 = initPose(1) / 2 * (1 + cos(EIGEN_PI*i / 19));
		pathOut(i, 1) = z2;
		pathOut(i, 0) = (z2 - parabolic(0, 2))*(z2 - parabolic(0, 2)) / 4 / parabolic(0, 0) + parabolic(0, 1);
	}

	//for (int i = 0; i < 20; i++)
	//{
	//	double z1 = initPose(0) / 2 * (1 + cos(EIGEN_PI*i / 19));
	//	pathOut(i, 0) = z1;
	//	pathOut(i, 1) = (z1 - parabolic(1, 1))*(z1 - parabolic(1, 1)) / 4 / parabolic(1, 0) + parabolic(1, 2);
	//}

	//for (int i = 0; i < 20; i++)
	//{
	//	double z1 = initPose(0) / 2 * (1 + cos(EIGEN_PI*i / 19));
	//	pathOut(i, 0) = z1;
	//	pathOut(i, 1) = cubic1(pathId, 0) + cubic1(pathId, 1)*z1 + cubic1(pathId, 2)*z1*z1 + cubic1(pathId, 3)*z1*z1*z1;
	//}

	//for (int i = 0; i < 20; i++)
	//{
	//	double z2 = initPose(1) / 2 * (1 + cos(EIGEN_PI*i / 19));
	//	pathOut(i, 1) = z2;
	//	pathOut(i, 0) = cubic2(pathId, 0) + cubic2(pathId, 1)*z2 + cubic2(pathId, 2)*z2*z2 + cubic2(pathId, 3)*z2*z2*z2;
	//}
	return pathOut;
}

void comCaspLensTest()
{
	// ComCaspTest
	int ch;
	ComCaspTest casp1;
	casp1.comConnect();
	casp1.setFocusVoltage(60);
	
	double caspFocus = 0;
	while (true)
	{
		if (_kbhit()){
			ch = _getch();
			std::cout << ch << std::endl;
			if (ch == 113) // q = caspFocus ++
			{
				caspFocus = casp1.getFocusVoltage();
				std::cout << "focus= " << caspFocus << std::endl;
				caspFocus++;
				if (caspFocus <= 69) { casp1.setFocusVoltage(caspFocus); }
			}
			else if (ch == 97) // a = caspFocus --
			{
				caspFocus = casp1.getFocusVoltage();
				std::cout << "focus= " << caspFocus << std::endl;
				caspFocus--;
				if (caspFocus >= 25) { casp1.setFocusVoltage(caspFocus); }
			}
			else if (ch == 27){ break; }
		}
	}

	casp1.comClose();
	system("pause");
	return;
}

void fromDesired2FactualTest()
{
	Eigen::Vector3d cartIniPos(-1000, 1500, 0);
	Eigen::Vector3d tarPos(0, 0, -80 * EIGEN_PI / 180);
	MatrixPath desiredPath = pathGenerator(cartIniPos, tarPos);
	//std::cout << desiredPath << std::endl;
	CartPara cartTest;
	cartTest.wheelsRadius = 50; // r=50mm
	cartTest.wheelsDistance = 560; // l=560mm
	cartTest.linVelLim = 50; // vlim=50mm/s
	cartTest.linAccLim = 20; // alim=20mm/s^2
	cartTest.angVelLim = 0.1; // theta'=0.1rad/s
	cartTest.angAccLim = 0.04; // theta''=0.04rad/s^2
	cartTest.communFreq = 10; // 10Hz
	MotionController mC(cartTest);
	/*MatrixPath desiredPath;
	desiredPath << 0, 0,
	1, 0,
	2, 0,
	3, 0,
	4, 0,
	5, 0,
	6, 0,
	7, 0,
	7, 1,
	7, 2,
	7, 3,
	7, 4,
	7, 5,
	8, 5,
	9, 5,
	10, 5,
	11, 5,
	12, 5,
	13, 5,
	14, 5;*/
	MatrixPath factualPath = mC.fromDesired2FactualPath(desiredPath, cartIniPos, tarPos);
	return;
}

void captureImage() {

	// open caspian
	ComCaspTest casp1;
	casp1.comConnect();
	
	// open camera
	GxCamTest GxHandler;
	if (!GxHandler.openDevice()) {
		return;
	}
	GxHandler.startSnap(); // GxHandler.imgFlow starts to update.
	
	casp1.setFocusVoltage(55);
	Sleep(500);
	GxHandler.setCheckSaveBmp();
	
	int ch;
	while (true)
	{
		if (_kbhit()) {
			ch = _getch();
			if (ch==113) { // q
				casp1.setFocusVoltage(60);
				Sleep(80);
				GxHandler.setCheckSaveBmp();
			}
			else if (ch == 27) { // esc
				break; 
			}
		}
	}

	casp1.comClose();

	GxHandler.stopSnap();
	GxHandler.closeDevice();
	return;
}

void markerDetectAndFollow() {

	// registe Detector
	Detector monitor;
	if (!monitor.getInitStatus()) {
		std::cout << "fail to registe Detector!" << std::endl;
		return;
	}
	else {
		std::cout << "monitor successfully connected!" << std::endl;
	}

	// from coarse to fine. get the best focal voltage.
	//bool coarseSuccess = false;
	//while (!coarseSuccess) {
	//	coarseSuccess = monitor.coarseToFine();
	//}
	//std::cout << "coarseToFine Success!" << std::endl;

	// pose estimate
	bool trackSuccess = false;
	while (!trackSuccess) {
		trackSuccess = monitor.estimatePose(true); // first estimate.
	}
	while (true) {
		// get another capturedImg
		trackSuccess = monitor.estimatePose();

		if (trackSuccess) {
			std::cout << "trackNextCaptured success!" << std::endl;
		}
		else {
			std::cout << "trackNextCaptured failed!" << std::endl;
			while (!trackSuccess) {
				trackSuccess = monitor.estimatePose(true);
			}
		}

		int ch;
		if (_kbhit()) { // if any key is pressed, _kbhit() = true.
			ch = _getch(); // get keyboard value
			if (ch == 27) { break; } // 27: esc
		}
	}
	return;
}

float sharpnessSingleImg(cv::Mat imgIn) {
	float sharpness = 0;

	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
	std::vector<std::vector<cv::Point2f>> markersCorners;
	std::vector<int> markersId;
	cv::aruco::detectMarkers(imgIn, dictionary, markersCorners, markersId);

	if (markersId.size() == 0) {
		return 0;
	}
	ArucoContainer arucoTemp(markersId.at(0), markersCorners.at(0));
	cv::Mat ROI(imgIn, arucoTemp.area);

	cv::Mat img_x, img_y, img_sobel;
	cv::Sobel(ROI, img_x, CV_16S, 1, 0); // be careful to use CV_16S here, maintaining useful information.
	cv::Sobel(ROI, img_y, CV_16S, 0, 1);
	cv::convertScaleAbs(img_x, img_x); // absolute value.
	cv::convertScaleAbs(img_y, img_y);
	cv::addWeighted(img_x, 0.5, img_y, 0.5, 0.0, img_sobel);

	float range[] = { 0, 256 }; // from 0 to 255
	int histBinNum = range[1] - range[0];
	const float* histRange = { range };
	cv::Mat hist;
	cv::calcHist(&img_sobel, 1, 0, cv::Mat(), hist, 1, &histBinNum, &histRange, true, false);

	int pixelNum = 0;
	for (int i = 95; i < histBinNum; i++) // i represents gradient; 95 = 255/2 * 3/4
	{
		float bin_val = hist.at<float>(i);
		sharpness += bin_val * i;
		pixelNum += bin_val;
	}
	sharpness = sharpness / pixelNum;

	return sharpness;
}

void sharpnessTest() {

	// open caspian
	ComCaspTest casp1;
	casp1.comConnect();

	// open camera
	GxCamTest GxHandler;
	if (!GxHandler.openDevice()) {
		return;
	}
	GxHandler.startSnap(); // GxHandler.imgFlow starts to update.

	float focusVoltage = 52;
	casp1.setFocusVoltage(focusVoltage);

	int ch;
	float sharpness = 0;
	std::ofstream ofs;
	ofs.open("text.txt", std::ios::out);
	while (true)
	{
		if (_kbhit()) {
			ch = _getch();
			if (ch == 97) { // a
				for (focusVoltage = 52; focusVoltage > 50; focusVoltage -= 0.1) {
					casp1.setFocusVoltage(focusVoltage);
					Sleep(200);
					cv::Mat imgTemp(GxHandler.getImgFlow());
					cv::imwrite(std::to_string(focusVoltage) + ".bmp", imgTemp);
					sharpness = sharpnessSingleImg(imgTemp);
					std::cout << focusVoltage << ": " << sharpness << "\n";
					ofs << focusVoltage << "\t" << sharpness << "\n";
				}
			}
			else if (ch == 27) { // esc
				break;
			}
		}
	}

	ofs << std::endl;
	ofs.close();

	casp1.comClose();

	GxHandler.stopSnap();
	GxHandler.closeDevice();
	return;
}


Corners getReprojectImagePoint(const std::vector<cv::Point3f>& object3DPoints, const cv::Vec3d& rvec, const cv::Vec3d& tvec)
{
	cv::Mat reprojectedPoints;
	cv::InputArray object3DPoints_x = (cv::InputArray)object3DPoints;
	cv::projectPoints(object3DPoints, rvec, tvec, cameraMatrix, distCoeffs, reprojectedPoints);

	Corners corners_res = Corners();
	size_t n = object3DPoints_x.rows() * object3DPoints_x.cols();
	for (size_t i = 0; i < n; i++)
	{
		corners_res.push_back(Corner(reprojectedPoints.at<cv::Vec2f>(i)[0], reprojectedPoints.at<cv::Vec2f>(i)[1]));
	}

	return corners_res;
}

/*draw axis and reprojected corners*/
void showBothResult(const cv::Mat& imgIn, const Corners& imgCnr, const Corners& coordi_crs,
	const cv::Vec3d& rvec, const cv::Vec3d& tvec, double cost_res, cv::Mat& img_resized)
{
	/*predefined parameters*/
	double resize_scale = 0.3;
	double text_scale = 1.5;
	int point_radius = 8;
	int line_width = 4;
	cv::Scalar color_arr[5] = { cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0), cv::Scalar(192, 112, 0), cv::Scalar(0, 255, 255) };
	// r, g, b

	cv::Mat img_return = imgIn;

	// draw corners
	for (const Corner& sc : imgCnr)
	{
		cv::circle(img_return, Corner(sc.x, sc.y), point_radius, color_arr[3], -1);
	}
	// draw coordinate system
	if (coordi_crs.size() == 4)
	{
		if ((coordi_crs.at(0).x > 0 && coordi_crs.at(0).x < img_return.cols - 1) &&
			(coordi_crs.at(0).y > 0 && coordi_crs.at(0).y < img_return.rows - 1) &&
			(coordi_crs.at(1).x > 0 && coordi_crs.at(1).x < img_return.cols - 1) &&
			(coordi_crs.at(1).y > 0 && coordi_crs.at(1).y < img_return.rows - 1) &&
			(coordi_crs.at(2).x > 0 && coordi_crs.at(2).x < img_return.cols - 1) &&
			(coordi_crs.at(2).y > 0 && coordi_crs.at(2).y < img_return.rows - 1) &&
			(coordi_crs.at(3).x > 0 && coordi_crs.at(3).x < img_return.cols - 1) &&
			(coordi_crs.at(3).y > 0 && coordi_crs.at(3).y < img_return.rows - 1))
		{
			cv::line(img_return, Corner(coordi_crs.at(0).x, coordi_crs.at(0).y), Corner(coordi_crs.at(1).x, coordi_crs.at(1).y), color_arr[0], line_width);
			cv::line(img_return, Corner(coordi_crs.at(0).x, coordi_crs.at(0).y), Corner(coordi_crs.at(2).x, coordi_crs.at(2).y), color_arr[1], line_width);
			cv::line(img_return, Corner(coordi_crs.at(0).x, coordi_crs.at(0).y), Corner(coordi_crs.at(3).x, coordi_crs.at(3).y), color_arr[2], line_width);
		}
	}

	std::string str1;
	if (cost_res < 100)
	{
		str1 = "position [" + std::to_string(tvec[0]) + "," + std::to_string(tvec[1]) + "," + std::to_string(tvec[2]) +
			"], orientation [" + std::to_string(rvec[0]) + "," + std::to_string(rvec[1]) + "," + std::to_string(rvec[2]) + "]";
	}
	cv::putText(img_return, str1, cv::Point_<int>(10, 50), cv::FONT_HERSHEY_COMPLEX, text_scale, color_arr[4], 2);

	cv::resize(img_return, img_resized, cv::Size(), resize_scale, resize_scale);

	return;
}

std::vector<std::vector<cv::Point2f>> subPixelCorners(const std::vector<std::vector<cv::Point2f>>& intACs, const cv::Mat& imgIn) {

	// corner region size.
	int HalfH = 4;
	int HalfW = 4;
	int winH = HalfH * 2 + 1;
	int winW = HalfW * 2 + 1;
	int winCnt = winH * winW;
	// iteration exit criteria
	int max_iters = 5;
	float eps = 0.000001; // 1e-6

	// gaussian weight mask
	cv::Mat weightMask = cv::Mat(winH, winW, CV_32FC1);
	for (int i = 0; i < winH; i++)
	{
		for (int j = 0; j < winW; j++)
		{
			float wx = (float)(j - HalfW) / HalfW;
			float wy = (float)(i - HalfH) / HalfH;
			float vx = exp(-wx * wx);
			float vy = exp(-wy * wy);
			weightMask.at<float>(i, j) = (float)(vx * vy);
		}
	}

	// for all the points
	std::vector<std::vector<cv::Point2f>> floatACs;
	for (const std::vector<cv::Point2f>& intAC : intACs) {
		std::vector<cv::Point2f > mC;

		for (const cv::Point2f& currPoint : intAC) {

			double a, b, c, bb1, bb2;
			cv::Mat subImg = cv::Mat::zeros(winH + 2, winW + 2, CV_8UC1); // +2 for sobel operation.
			cv::Point2f iterPoint = currPoint;

			// start iteration
			int iterCnt = 0;
			float err = 1.0;
			while (err > eps && iterCnt < max_iters) {
				a = b = c = bb1 = bb2 = 0;

				int initx = int(iterPoint.x + 0.5f) - (winW + 1) / 2;
				int inity = int(iterPoint.y + 0.5f) - (winH + 1) / 2;
				if (initx < 0 || inity < 0 || (initx + winW + 2) >= imgIn.cols || (inity + winH + 2) >= imgIn.rows) {
					break;
				}
				cv::Rect imgROI(initx, inity, winW + 2, winH + 2);
				subImg = imgIn(imgROI).clone();

				for (int i = 0; i < winH; i++) {
					for (int j = 0; j < winW; j++) {
						// read gaussian weight.
						double m = weightMask.at<float>(i, j);
						// soble operation.
						double sobelx = static_cast<double>(subImg.at<uchar>(i + 1, j + 2) - subImg.at<uchar>(i + 1, j));
						double sobely = static_cast<double>(subImg.at<uchar>(i + 2, j + 1) - subImg.at<uchar>(i, j + 1));
						double gxx = sobelx * sobelx * m;
						double gxy = sobelx * sobely * m;
						double gyy = sobely * sobely * m;
						a += gxx;
						b += gxy;
						c += gyy;
						// p_i position.
						double px = static_cast<double>(j - HalfW);
						double py = static_cast<double>(i - HalfH);

						bb1 += gxx * px + gxy * py;
						bb2 += gxy * px + gyy * py;
					}
				}
				double det = a * c - b * b;
				if (fabs(det) <= DBL_EPSILON * DBL_EPSILON)
					break;
				// inverse
				double invA = c / det;
				double invC = a / det;
				double invB = -b / det;
				// new point
				cv::Point2f newPoint;
				newPoint.x = (float)(initx + HalfW + 1 + invA * bb1 + invB * bb2);
				newPoint.y = (float)(inity + HalfH + 1 + invB * bb1 + invC * bb2);
				// update err.
				err = (newPoint.x - iterPoint.x) * (newPoint.x - iterPoint.x) + (newPoint.y - iterPoint.y) * (newPoint.y - iterPoint.y);
				// update new point.
				iterPoint = newPoint;
				iterCnt++;
				if (iterPoint.x < 0 || iterPoint.x >= imgIn.cols || iterPoint.y < 0 || iterPoint.y >= imgIn.rows)
					break;
			}

			// verify convergence
			if (fabs(iterPoint.x - currPoint.x) > 1 || fabs(iterPoint.y - currPoint.y) > 1) {
				iterPoint = currPoint;
			}

			// save the results.
			mC.emplace_back(iterPoint);
		}

		floatACs.emplace_back(mC);
	}

	return floatACs;
}

void relativePoseEstimate() {
	// only 3 arucos' relative pose can be estimated every time.
	//std::vector<int> estimateIds = { 0,1,2 }; // id number from left to right. 
	std::vector<int> estimateIds = { 1,3,4,5,7 };

	cv::Mat imageCopy;
	std::string prefix="E:\\sjtu\\autoPark\\code\\calibrate\\relativePoseEstimate\\5marker\\";
	std::ofstream ofs; // 5n X 6.
	ofs.open(prefix+"relativePoseEstimate.txt", std::ios::out);
	for (int imgName = 0; imgName < 280; imgName++) {
		std::string imgStr = prefix + std::to_string(imgName) + ".bmp";
		std::string imgStrOut = prefix + std::to_string(imgName) + "_.bmp";
		imageCopy = cv::imread(imgStr);

		cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
		std::vector<std::vector<cv::Point2f>> markersCornersRaw;
		cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
		parameters->minMarkerPerimeterRate = 0.1; // the minimum pixel size of a marker, specified relative to the maximum dimension of the input image.
		parameters->maxMarkerPerimeterRate = 0.8; // the minimum pixel size of a marker...
		parameters->minMarkerDistanceRate = 0.01; // Minimum distance between any pair of corners from two different markers.
		std::vector<int> markersIdRaw;
		cv::aruco::detectMarkers(imageCopy, dictionary, markersCornersRaw, markersIdRaw, parameters);

		// if at least one marker detected
		if (markersIdRaw.size() == 0) {
			std::cout << "no marker..." << std::endl;
			continue;
		}
		std::vector<std::vector<cv::Point2f>> markersCorners;
		std::vector<int> markersId;
		auto first = markersIdRaw.begin();
		auto ff = markersCornersRaw.begin();
		while (first != markersIdRaw.end()) {
			bool unique=true;
			auto second = first + 1;
			auto ss = ff + 1;
			while (second != markersIdRaw.end()) {
				if (*second == *first) {
					unique = false;
					if ((fabs(ff->at(0).x - ff->at(1).x) + fabs(ff->at(0).y - ff->at(3).y)) > (fabs(ss->at(0).x - ss->at(1).x) + fabs(ss->at(0).y - ss->at(3).y))) {
						markersId.emplace_back(*second);
						markersCorners.emplace_back(*ss);
					}
					else {
						markersId.emplace_back(*first);
						markersCorners.emplace_back(*ff);
					}
					markersIdRaw.erase(second);
					markersCornersRaw.erase(ss);
					break;
				}
				second++;
				ss++;
			}
			if (unique) {
				markersId.emplace_back(*first);
				markersCorners.emplace_back(*ff);
			}
			first++;
			ff++;
		}

		markersCorners = subPixelCorners(markersCorners, imageCopy); // subpixel location

		cv::aruco::drawDetectedMarkers(imageCopy, markersCorners, markersId); // draw the results
		std::vector<cv::Vec3d> rvecs, tvecs;

		cv::aruco::estimatePoseSingleMarkers(markersCorners, markerSize/1000, cameraMatrix, distCoeffs, rvecs, tvecs); //求解旋转矩阵rvecs和平移矩阵tvecs
		std::cout<<"R :"<<rvecs[0]<< std::endl;
		std::cout << "T :" << tvecs[0] << std::endl;

		for (int i = 0; i < estimateIds.size(); i++) {
			std::vector<int>::iterator iter = std::find(markersId.begin(), markersId.end(), estimateIds[i]);
			if (iter == markersId.end())
			{
				ofs << "0 0 0 0 0 0" << "\n";
			}
			else {
				int idNumTemp = std::distance(markersId.begin(), iter);
				ofs << rvecs[idNumTemp][0] << '\t' << rvecs[idNumTemp][1] << '\t' << rvecs[idNumTemp][2] << '\t' <<
					tvecs[idNumTemp][0] << '\t' << tvecs[idNumTemp][1] << '\t' << tvecs[idNumTemp][2] << "\n";
			}
		}

		// draw axis for each marker
		for (int i = 0; i < markersId.size(); i++)
			cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
		

		cv::resize(imageCopy, imageCopy, cv::Size(), 0.3, 0.3);
		cv::imwrite(imgStrOut, imageCopy);
		//cv::waitKey(1);
	}
	ofs << std::endl;

}

void poseEstimateTest() {
	// three points

	//std::vector<int> estimateIds = {0,1,2}; // id number from left to right. 

	//// open caspian
	//ComCaspTest casp1;
	//casp1.comConnect();

	//// open camera
	//GxCamTest GxHandler;
	//if (!GxHandler.openDevice()) {
	//	return;
	//}
	//GxHandler.startSnap(); // GxHandler.imgFlow starts to update.

	//casp1.setFocusVoltage(46.5); // 标定用的镜头电压
	//Sleep(200);
	
	cv::Mat imageCaptured;
	//std::ofstream ofs;
	std::string prefix = "E:\\sjtu\\autoPark\\code\\autoPark\\autoPark\\poseEstimate\\";
	//ofs.open(prefix+"poseEstimateLeft.txt", std::ios::out);

	for (int num_i = 0; num_i < 1; num_i++) {
		cv::Mat imageCopy;
		std::string imgStr = prefix + std::to_string(num_i) + ".bmp";
		std::string imgStrOut = prefix + std::to_string(num_i) + "_.bmp";
		imageCaptured = cv::imread(imgStr);
		//cv::resize(imageCaptured, imageCaptured, cv::Size(), 0.5, 0.5);
		//imageCaptured = GxHandler.getImgFlow(); // 抓取视频中的一张照片
		imageCaptured.copyTo(imageCopy);
		cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
		std::vector<std::vector<cv::Point2f>> markersCorners;
		std::vector<int> markersId;
		cv::aruco::detectMarkers(imageCaptured, dictionary, markersCorners, markersId);

		if (markersId.size() == 0) {
			std::cout << "no marker..." << std::endl;
			continue;
		}

		// if at least one marker detected
		//cv::aruco::drawDetectedMarkers(imageCopy, markersCorners, markersId);//绘制检测到的靶标的框

		/*single estimate*/
		//std::vector<cv::Vec3d> rvecs, tvecs;
		//cv::aruco::estimatePoseSingleMarkers(markersCorners, 0.08, cameraMatrix, distCoeffs, rvecs, tvecs);
		//for (int i = 0; i < markersId.size(); i++) {
		//	std::cout << markersId[i];
		//	std::cout << "R :" << rvecs[i] << std::endl;
		//	std::cout << "T :" << tvecs[i] << std::endl;
		//}

		std::vector<cv::Point3f> pointsM;
		std::vector<cv::Point2f> pointsP;
		for (int i = 0; i < estimateIds.size(); i++) {
			std::vector<int>::iterator iter = std::find(markersId.begin(), markersId.end(), estimateIds[i]);
			if (iter != markersId.end()) {
				int idNumTemp = std::distance(markersId.begin(), iter);

				Eigen::Vector4f pointX_0(-markerSize / 2, markerSize / 2, 0.0, 1.0);
				Eigen::Vector4f pointX_1(markerSize / 2, markerSize / 2, 0.0, 1.0);
				Eigen::Vector4f pointX_2(markerSize / 2, -markerSize / 2, 0.0, 1.0);
				Eigen::Vector4f pointX_3(-markerSize / 2, -markerSize / 2, 0.0, 1.0);
				Eigen::Vector4f pointM_0 = poseRelativeX[i] * pointX_0;
				Eigen::Vector4f pointM_1 = poseRelativeX[i] * pointX_1;
				Eigen::Vector4f pointM_2 = poseRelativeX[i] * pointX_2;
				Eigen::Vector4f pointM_3 = poseRelativeX[i] * pointX_3;
				pointsM.emplace_back(pointM_0(0), pointM_0(1), pointM_0(2));
				pointsM.emplace_back(pointM_1(0), pointM_1(1), pointM_1(2));
				pointsM.emplace_back(pointM_2(0), pointM_2(1), pointM_2(2));
				pointsM.emplace_back(pointM_3(0), pointM_3(1), pointM_3(2));
				//std::cout << pointsM.at(0+4*i) << std::endl;
				//std::cout << pointsM.at(1+4*i) << std::endl;
				//std::cout << pointsM.at(2+4*i) << std::endl;
				//std::cout << pointsM.at(3+4*i) << std::endl;

				/*std::cout << markersCorners[idNumTemp].at(0) << std::endl;
				std::cout << markersCorners[idNumTemp].at(1) << std::endl;
				std::cout << markersCorners[idNumTemp].at(2) << std::endl;
				std::cout << markersCorners[idNumTemp].at(3) << std::endl;*/
				pointsP.emplace_back(markersCorners[idNumTemp].at(0));
				pointsP.emplace_back(markersCorners[idNumTemp].at(1));
				pointsP.emplace_back(markersCorners[idNumTemp].at(2));
				pointsP.emplace_back(markersCorners[idNumTemp].at(3));
			}
		}

		cv::Vec3d rvec, tvec;
		cv::solvePnP(pointsM, pointsP, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_EPNP); // r and t: Tcam_M
		//ofs << rvec[0]<< '\t' << rvec[1] << '\t' << rvec[2] << '\t' << tvec[0] << '\t' << tvec[1] << '\t' << tvec[2] << "\n";

		// draw results
		Corners reproPoints;
		reproPoints = getReprojectImagePoint(pointsM, rvec, tvec); // r and t: Tcam_M
		cv::Mat imgResized;
		std::vector<cv::Point3f> coordi_crs3d;
		/*cv::Mat rotm;
		cv::Rodrigues(rvec, rotm);*/
		//std::cout << rotm << std::endl;
		coordi_crs3d.emplace_back(0.0, 0.0, 0.0);
		coordi_crs3d.emplace_back(60.0, 0.0, 0.0);
		coordi_crs3d.emplace_back(0.0, 60.0, 0.0);
		coordi_crs3d.emplace_back(0.0, 0.0, 60.0);
		Corners coordi_crs;
		coordi_crs = getReprojectImagePoint(coordi_crs3d, rvec, tvec); // r and t: Tcam_M
		showBothResult(imageCopy, reproPoints, coordi_crs, rvec, tvec, 5, imgResized);
		cv::imwrite(imgStrOut, imgResized);
	}

	//ofs << std::endl;
	//ofs.close();

	//casp1.comClose();

	//GxHandler.stopSnap();
	//GxHandler.closeDevice();

	return;
}


int main()
{

	if (!loadConfig())
		return -1;

	//comCaspLensTest();
	//testCornerDetect();

	//captureImage();

	//sharpnessTest();

	markerDetectAndFollow();
	//cv::Mat imgOrigin;
	//imgOrigin = cv::imread("E:\\projects\\Project1\\Project1\\46.7.jpg");
	//cv::Mat imgCopy(imgOrigin, cv::Rect(10, 10, ceil(imgOrigin.cols/2), ceil(imgOrigin.rows/2)));
	//imgCopy = imgOrigin.clone();
	//cv::circle(imgCopy, Corner(27, 33), 1, {255,0,0}, -1);
	//cv::resize(imgCopy, imgCopy, cv::Size(), 0.7, 0.7);

	//relativePoseEstimate();

	
	//poseEstimateTest();
	

	//cv::Mat imageCaptured = cv::imread("E:\\sjtu\\autoPark\\code\\0.bmp");
	//cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
	//std::vector<std::vector<cv::Point2f>> markersCorners;
	//std::vector<int> markersId;
	//cv::aruco::detectMarkers(imageCaptured, dictionary, markersCorners, markersId);

	//// if at least one marker detected
	//if (markersId.size() > 0) {
	//	cv::aruco::drawDetectedMarkers(imageCaptured, markersCorners, markersId);//绘制检测到的靶标的框
	//	std::vector<cv::Vec3d> rvecs, tvecs;
	//	cv::aruco::estimatePoseSingleMarkers(markersCorners, 0.08, cameraMatrix, distCoeffs, rvecs, tvecs);//求解旋转矩阵rvecs和平移矩阵tvecs
	//	std::cout << "R :" << rvecs[0] << std::endl;
	//	std::cout << "T :" << tvecs[0] << std::endl;

	//	// draw axis for each marker
	//	for (int i = 0; i < markersId.size(); i++) {
	//		cv::aruco::drawAxis(imageCaptured, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
	//	}
	//	// rectify each corner.
	//	//Detector monitor;
	//	//for (const cv::Point2f& markerCorner : markersCorners.at(0)) {
	//	//	Corner cornerTemp = monitor.rectifyOneCorner(markerCorner);
	//	//	std::cout << "原始图形坐标：" << markerCorner.x << "\t" << markerCorner.y << "\n";
	//	//	std::cout << "去畸变后坐标：" << cornerTemp.x << "\t" << cornerTemp.y << std::endl;

	//}

	//cv::imwrite("E:\\sjtu\\autoPark\\code\\0_.bmp", imageCaptured);
	////cv::waitKey(0);

	return 0;
}