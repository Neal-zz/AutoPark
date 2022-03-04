#include "MotionController.h"
#include "ComCaspTest.h"
#include "GxCamTest.h"
#include "Detector.h"
#include "Optimizator.h"

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
		Eigen::Vector2d cartVelNi(std::min(disErr, cartTest.linVelLim), abs(angErr)>cartTest.angVelLim ? cartTest.angVelLim*angErr / abs(angErr) : angErr);
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
		if (abs(initOri(i) - EIGEN_PI / 2) < 0.001 || abs(initOri(i) + EIGEN_PI / 2) < 0.001 || abs(initPose(0)) < 0.001) // other criterions are discussed later.
			cubic1State(i) = 1;
		cubic1(i, 0) = 0;
		cubic1(i, 1) = tan(finalPose(2));
		cubic1(i, 2) = (3 * initPose(1) - initPose(0)*(tan(initOri(i)) + 2 * tan(finalPose(2)))) / initPose(0) / initPose(0);
		cubic1(i, 3) = (initPose(0)*(tan(initOri(i)) + tan(finalPose(2))) - 2 * initPose(1)) / initPose(0) / initPose(0) / initPose(0);
		if (abs(initOri(i)) < 0.001 || abs(abs(initOri(i)) - EIGEN_PI) < 0.001 || abs(initPose(1)) < 0.001)
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
	// 第一版：只关注 ROI1.

	// registe Detector
	Detector monitor;
	if (!monitor.getInitStatus()) {
		std::cout << "fail to registe Detector!" << std::endl;
		return;
	}
	else {
		std::cout << "monitor successfully connected!" << std::endl;
	}
	Sleep(1000);

	// from coarse to fine
	bool coarseSuccess = false;
	while (!coarseSuccess) {
		coarseSuccess = monitor.coarseToFine();
	}
	std::cout << "coarseToFine Success!" << std::endl;

	bool trackSuccess = false;
	while (true) {
		// get another capturedImg, track aruco1
		trackSuccess = monitor.trackNextCaptured();

		if (trackSuccess) {
			std::cout << "trackNextCaptured success!" << std::endl;
		}
		else {
			std::cout << "trackNextCaptured failed!" << std::endl;
			bool coarseSuccess = false;
			while (!coarseSuccess) {
				coarseSuccess = monitor.coarseToFine();
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

	float focusVoltage = 49;
	casp1.setFocusVoltage(focusVoltage);

	int ch;
	float sharpness = 0;
	std::ofstream ofs;
	ofs.open("text.txt", std::ios::out);
	while (true)
	{
		if (_kbhit()) {
			ch = _getch();
			if (ch == 113) { // q
				focusVoltage += 0.1;
				casp1.setFocusVoltage(focusVoltage);
				Sleep(200);
				sharpness = sharpnessSingleImg(GxHandler.getImgFlow());
				std::cout << focusVoltage << ": " << sharpness << "\n";
			}
			if (ch == 97) { // a
				for (focusVoltage = 49; focusVoltage > 45; focusVoltage -= 0.1) {
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

int main()
{
	//comCaspLensTest();
	//testCornerDetect();

	//captureImage();

	//sharpnessTest();

	markerDetectAndFollow();

	return 0;
}