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
	casp1.setFocusNum(40);
	double caspFocus = 0;
	while (1 == 1)
	{
		if (_kbhit()){
			ch = _getch();
			if (ch == 81) // q = +
			{
				caspFocus = casp1.getFocusNum();
				std::cout << "focus= " << caspFocus << std::endl;
				caspFocus++;
				if (caspFocus <= 69) { casp1.setFocusNum(caspFocus); }
			}
			else if (ch == 65) // a = -
			{
				caspFocus = casp1.getFocusNum();
				std::cout << "focus= " << caspFocus << std::endl;
				caspFocus--;
				if (caspFocus >= 25) { casp1.setFocusNum(caspFocus); }
			}
			else if (ch == 27){ break; }
		}
	}
	system("pause");
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

void testCornerDetect() {
	auto t1 = tic();

	cv::Mat img = cv::imread("markerFar7.bmp");  // CV_8UC3
	if (!img.data)
	{
		printf(" No image data \n ");
		return;
	}
	cv::cvtColor(img, img, cv::COLOR_BGR2GRAY); // CV_8UC1
	img.convertTo(img, CV_32FC1, 1.0 / 255.0); // CV_32FC1

	Detector detector(img.size());
	QRsTemplate QRs = detector.process(img);

	toc(t1, "t1");

	cv::Mat imgColor;
	
	cv::cvtColor(img, imgColor, cv::COLOR_GRAY2BGR);
	imgColor.convertTo(imgColor, CV_8UC3, 255.0);
	detector.showResult("cor", QRs, imgColor);
	return;
}

void captureImage() {
	GxCamTest GxHandler;
	bool success = GxHandler.openDevice();
	if (!success) {
		std::cout << "failed to open device..." << std::endl;
		return;
	}

	GxHandler.startSnap();
	while (true) {
		int ch;
		if (_kbhit()) { //如果有按键按下，则_kbhit()函数返回真
			ch = _getch(); // get keyboard value
			//std::cout << ch << std::endl;
			if (ch == 27) { break; } // 27: esc
			else if (ch == 119) { GxHandler.setCheckSaveBmp(); } // 119: w
		}
	}

	GxHandler.stopSnap();
	GxHandler.closeDevice();
	return;
}

int main()
{
	//testCornerDetect();

	captureImage();

	return 0;
}