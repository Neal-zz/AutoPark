#include "MotionController.h"
#include <Eigen\Dense>
#include <iostream>
#include <fstream>


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


int main()
{
	/*forwardControl();*/

	CartPara cartTest;
	cartTest.wheelsRadius = 0.15;
	cartTest.wheelsDistance = 0.4;
	cartTest.linVelLim = 1.2;
	cartTest.linAccLim = 0.6;
	cartTest.angVelLim = 1.5;
	cartTest.angAccLim = 0.8;
	cartTest.communFreq = 10;
	MotionController mC(cartTest);
	Eigen::Vector3d cartIniPos(0, 0, -EIGEN_PI/6);
	Eigen::Vector3d tarPos(14, 5, 0);
	MatrixPath desiredPath;
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
		14, 5;
	mC.generatePath(desiredPath, cartIniPos, tarPos);

	return 0;
}