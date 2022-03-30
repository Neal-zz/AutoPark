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
//void forwardControlTest() {
//	CartPara cartTest;
//	cartTest.wheelsRadius = 50;
//	cartTest.wheelsDistance = 560;
//	cartTest.linVelLim = 50;
//	cartTest.linAccLim = 20;
//	cartTest.angVelLim = 0.1;
//	cartTest.angAccLim = 0.04;
//	cartTest.communFreq = 10;
//
//	MotionController mC(cartTest);
//	mC.forwardControl();
//}


void closedLoopControlTest() {
	/* create motionController*/
	CartPara cartPara;
	cartPara.wheelsRadius = 50;
	cartPara.wheelsDistance = 560;
	cartPara.linVelLim = 50;
	cartPara.linAccLim = 20;
	cartPara.angVelLim = 0.1;
	cartPara.angAccLim = 0.04;
	cartPara.communFreq = 10;
	cartPara.r = 220;
	cartPara.l1 = 400;
	cartPara.w = 600;
	cartPara.l2 = 1000;
	MotionController mC(cartPara);
	
	/* open the camera. open the detector. register Detector*/
	// open caspian
	std::shared_ptr<ComCaspTest> caspian = std::make_shared<ComCaspTest>();
	/*caspian->comConnect();
	caspian->setFocusVoltage(49.8);
	if (!caspian->getComStatus()) {
		std::cout << "closedLoopControlTest(): caspian open error!" << std::endl;
		return;
	}*/
	// open camera
	std::shared_ptr<GxCamTest> GxCamera = std::make_shared<GxCamTest>();
	//if (!GxCamera->openDevice()) {
	//	std::cout << "closedLoopControlTest(): GxCamera open error!" << std::endl;
	//	return;
	//}
	//GxCamera->startSnap(); // GxHandler.imgFlow starts to update.
	//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	// register Detector
	Detector monitor(caspian, GxCamera);

	/* register trocar, trolley planeand collid plane. */
	// register trocar. needs to be done...
	mC.registerTrocar();
	// register collide plane
	std::vector<cv::Point3f> collidePlane;
	collidePlane.emplace_back(cv::Point3f(0, 0, 1200));
	collidePlane.emplace_back(cv::Point3f(0, 2000, 1200));
	collidePlane.emplace_back(cv::Point3f(-1000, 0, 1000.0));
	//collidePlane = monitor.registerPlane();
	mC.registerCollidePlane(collidePlane);
	// register trolley plane
	std::vector<cv::Point3f> motionPlane;
	motionPlane.emplace_back(cv::Point3f(1, 0, 1));
	motionPlane.emplace_back(cv::Point3f(1, 0, 2));
	motionPlane.emplace_back(cv::Point3f(0, 0, 3));
	//motionPlane = monitor.registerPlane();
	mC.registerTrolleyPlane(motionPlane);
	mC.updateTarPos();
	//mC.updateCartCurrPos(mC.getTarPos());  // test...

	/* generate roadSign*/
	// detect trolley
	bool coarseSuccess = false;
	//while (!coarseSuccess) {
	//	coarseSuccess = monitor.coarseToFine(46.8, 50.9);
	//}
	//std::cout << "coarseToFine Success!" << std::endl;
	bool trackSuccess = false;
	//while (!trackSuccess) {
	//	trackSuccess = monitor.estimatePose(true); // true: first estimate.
	//}
	// road sign
	pose3d initial_pose;
	initial_pose.tvec = cv::Vec3d(1500, 0, 5000);
	initial_pose.rvec = cv::Vec3d(2.5, 0, -1.5);
	mC.registerCartIniPos(initial_pose);  // test.....
	//mC.registerCartIniPos(monitor.getPose());
	mC.pathGenerator();  // pathRaw has been selected to be the roadSign.
	//std::vector<Eigen::Vector2d> pathRaw = mC.getRoadSign();
	std::vector<Eigen::Vector2d> factualPath = mC.fromDesired2FactualPath();
	mC.updateImg(factualPath);

	/* control loop*/
	double disThresh = 10; // 10mm
	int dpSign = 1; // desiredPath sign post (from 0-19).
	//Eigen::Vector2d tarPosi(pathRaw.at(dpSign)[0], pathRaw.at(dpSign)[1]);
	Eigen::Vector3d cartPosi(mC.getCartCurrPos());
	pose3d cartPosRaw;
	Eigen::Vector2d cartVeli(Eigen::Vector2d::Zero());
	Eigen::Vector2d wheelsVeli(Eigen::Vector2d::Zero());
	Eigen::Vector3d cartVel3i(Eigen::Vector3d::Zero());
	// while doesn't reach the target position.
	//while ( pow(cartPosi(0) - pathRaw.at(pathRaw.size()-1)[0], 2) + pow(cartPosi(1) - pathRaw.at(pathRaw.size()-1)[1], 2) > disThresh * disThresh) {
	//	// track another capturedImg and update cartPosi.
	//	trackSuccess = monitor.estimatePose();
	//	if (trackSuccess) {
	//		std::cout << "trackNextCaptured success!" << std::endl;
	//	}
	//	else {
	//		std::cout << "trackNextCaptured failed!" << std::endl;
	//		while (!trackSuccess) {
	//			trackSuccess = monitor.estimatePose(true);
	//		}
	//	}
	//	cartPosRaw = monitor.getPose();
	//	mC.updateCartCurrPos(cartPosRaw);  // used for imgshow.
	//	cartPosi = mC.getCartCurrPos();

	//	
	//	// update tarPosi when get close to the previous one.
	//	double disErr = sqrt(pow(cartPosi(0) - tarPosi(0), 2) + pow(cartPosi(1) - tarPosi(1), 2));
	//	if (dpSign < (pathRaw.size()-1)) {  // < 19
	//		double preErr = sqrt(pow(cartPosi(0) - pathRaw.at(dpSign - 1)[0], 2) + pow(cartPosi(1) - pathRaw.at(dpSign - 1)[1], 2));
	//		/*parameter 0.5 needs to be adjusted.*/
	//		if (preErr > disErr)
	//		{
	//			//roadSign.emplace_back(Eigen::Vector2d(cartPosi(0), cartPosi(1)));
	//			dpSign++;
	//			tarPosi = pathRaw.at(dpSign);  // update tarPosi.
	//			disErr = sqrt(pow(cartPosi(0) - tarPosi(0), 2) + pow(cartPosi(1) - tarPosi(1), 2));
	//			//std::cout << cartPosi(0) << cartPosi(1) << std::endl;
	//		}
	//	}
	//	double angErr = atan2(tarPosi(1) - cartPosi(1), tarPosi(0) - cartPosi(0)) - cartPosi(2);
	//	if (angErr > EIGEN_PI) {
	//		angErr -= 2 * EIGEN_PI;
	//	}
	//	else if (angErr < -EIGEN_PI) {
	//		angErr += 2 * EIGEN_PI;
	//	}

	//	// comute next velocity.
	//	/* kp=1*/
	//	Eigen::Vector2d cartVelNi((std::min)({ disErr, cartPara.linVelLim }),
	//		fabs(angErr) > cartPara.angVelLim ? cartPara.angVelLim * angErr / fabs(angErr) : angErr);
	//	if ((cartVelNi(0) - cartVeli(0)) * cartPara.communFreq > cartPara.linAccLim)
	//		cartVelNi(0) = cartVeli(0) + cartPara.linAccLim / cartPara.communFreq;
	//	else if ((cartVelNi(0) - cartVeli(0)) * cartPara.communFreq < -cartPara.linAccLim)
	//		cartVelNi(0) = cartVeli(0) - cartPara.linAccLim / cartPara.communFreq;
	//	if ((cartVelNi(1) - cartVeli(1)) * cartPara.communFreq > cartPara.angAccLim)
	//		cartVelNi(1) = cartVeli(1) + cartPara.angAccLim / cartPara.communFreq;
	//	else if ((cartVelNi(1) - cartVeli(1)) * cartPara.communFreq < -cartPara.angAccLim)
	//		cartVelNi(1) = cartVeli(1) - cartPara.angAccLim / cartPara.communFreq;
	//	cartVeli = cartVelNi;
	//	wheelsVeli = mC.fromCartVel2WheelsVel2(cartVeli);

	//	// send wheelsVeli





	//	int ch;
	//	if (_kbhit()) {  // if any key is pressed, _kbhit() = true.
	//		ch = _getch();  // get keyboard value
	//		if (ch == 27) { break; }  // 27: esc
	//	}
	//}


	/* close caspian and camera.*/ 
	caspian->comClose();
	GxCamera->stopSnap();
	GxCamera->closeDevice();

}

/* focal voltage control test.*/
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
		if (_kbhit()) {
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
			else if (ch == 27) { break; }
		}
	}

	casp1.comClose();
	system("pause");
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
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	GxHandler.setCheckSaveBmp();

	int ch;
	while (true)
	{
		if (_kbhit()) {
			ch = _getch();
			if (ch == 113) { // q
				casp1.setFocusVoltage(60);
				std::this_thread::sleep_for(std::chrono::milliseconds(80));
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

	// open caspian
	std::shared_ptr<ComCaspTest> caspian = std::make_shared<ComCaspTest>();
	caspian->comConnect();
	caspian->setFocusVoltage(49.8);
	if (!caspian->getComStatus()) {
		std::cout << "markerDetectAndFollor(): caspian open error!" << std::endl;
		return;
	}

	// open camera
	std::shared_ptr<GxCamTest> GxCamera = std::make_shared<GxCamTest>();
	if (!GxCamera->openDevice()) {
		std::cout << "markerDetectAndFollor(): GxCamera open error!" << std::endl;
		return;
	}
	GxCamera->startSnap(); // GxHandler.imgFlow starts to update.
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	// registe Detector
	Detector monitor(caspian, GxCamera);

	// from coarse to fine. get the best focal voltage.
	bool coarseSuccess = false;
	while (!coarseSuccess) {
		coarseSuccess = monitor.coarseToFine(46.8, 50.9);
	}
	std::cout << "coarseToFine Success!" << std::endl;
	
	/*std::cout << caspian->getFocusVoltage() << std::endl;
	cv::Mat imgT(GxCamera->getImgFlow());
	cv::imwrite("324.bmp", imgT);*/
	// pose estimate
	//bool trackSuccess = false;
	//while (!trackSuccess) {
	//	trackSuccess = monitor.estimatePose(true); // first estimate.
	//}
	//while (true) {
	//	// get another capturedImg
	//	trackSuccess = monitor.estimatePose();

	//	if (trackSuccess) {
	//		std::cout << "trackNextCaptured success!" << std::endl;
	//	}
	//	else {
	//		std::cout << "trackNextCaptured failed!" << std::endl;
	//		while (!trackSuccess) {
	//			trackSuccess = monitor.estimatePose(true);
	//		}
	//	}

	//	int ch;
	//	if (_kbhit()) { // if any key is pressed, _kbhit() = true.
	//		ch = _getch(); // get keyboard value
	//		if (ch == 27) { break; } // 27: esc
	//	}
	//}



	// close caspian
	caspian->comClose();

	// close camera
	GxCamera->stopSnap();
	GxCamera->closeDevice();

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
					std::this_thread::sleep_for(std::chrono::milliseconds(200));
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
						double px = static_cast<double>(j) - HalfW;
						double py = static_cast<double>(i) - HalfH;

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
	std::string prefix = "E:\\sjtu\\autoPark\\code\\calibrate\\relativePoseEstimate\\5marker\\";
	std::ofstream ofs; // 5n X 6.
	ofs.open(prefix + "relativePoseEstimate.txt", std::ios::out);
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
			bool unique = true;
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

		cv::aruco::estimatePoseSingleMarkers(markersCorners, markerSize / 1000, cameraMatrix, distCoeffs, rvecs, tvecs); //求解旋转矩阵rvecs和平移矩阵tvecs
		std::cout << "R :" << rvecs[0] << std::endl;
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

	//forwardControlTest();
	closedLoopControlTest();
	//comCaspLensTest();
	//testCornerDetect();

	//captureImage();


	//sharpnessTest();

	
	//markerDetectAndFollow();
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
