#include "MotionController.h"




MotionController::MotionController(const CartPara& cartParaIn)
	:cartPara(cartParaIn), cartIniPos(Eigen::Vector3d::Zero()), tarPos(Eigen::Vector3d::Zero()),
	cartCurrVel(Eigen::Vector2d::Zero()), wheelsVel(Eigen::Vector2d::Zero()), cartCurrPos(Eigen::Vector3d::Zero())
{
}

Eigen::Vector2d MotionController::fromCartVel2WheelsVel2() const
{
	Eigen::Vector2d wheelsVelOut; // (vR, vL).
	wheelsVelOut << cartCurrVel(0) + cartCurrVel(1)*cartPara.wheelsDistance / 2,
		cartCurrVel(0) - cartCurrVel(1)*cartPara.wheelsDistance / 2;
	return wheelsVelOut;
}

Eigen::Vector2d MotionController::fromCartVel2WheelsVel2(const Eigen::Vector2d& cartCurrVelIn) const
{
	Eigen::Vector2d wheelsVelOut; // (vR, vL).
	wheelsVelOut << cartCurrVelIn(0) + cartCurrVelIn(1)*cartPara.wheelsDistance / 2,
		cartCurrVelIn(0) - cartCurrVelIn(1)*cartPara.wheelsDistance / 2;
	return wheelsVelOut;
}

Eigen::Vector3d MotionController::fromWheelsVel2CartVel3() const
{
	Eigen::Vector3d cartVel3; // (xDot, yDot, thetaDot).
	cartVel3 << cos(cartCurrPos(2))*(wheelsVel(0) + wheelsVel(1)) / 2,
		sin(cartCurrPos(2))*(wheelsVel(0) + wheelsVel(1)) / 2,
		(wheelsVel(0) - wheelsVel(1)) / cartPara.wheelsDistance;
	return cartVel3;
}

Eigen::Vector3d MotionController::fromWheelsVel2CartVel3(
	const Eigen::Vector3d& cartCurrPosIn, const Eigen::Vector2d& wheelsVelIn) const
{
	Eigen::Vector3d cartVel3; // (xDot, yDot, thetaDot).
	cartVel3 << cos(cartCurrPosIn(2))*(wheelsVelIn(0) + wheelsVelIn(1)) / 2,
		sin(cartCurrPosIn(2))*(wheelsVelIn(0) + wheelsVelIn(1)) / 2,
		(wheelsVelIn(0) - wheelsVelIn(1)) / cartPara.wheelsDistance;
	return cartVel3;
}

MatrixPath MotionController::generatePath(const MatrixPath& desiredPath, const Eigen::Vector3d& iniPosIn, const Eigen::Vector3d& tarPosIn) const
{
	MatrixPath factualPath = Eigen::Matrix<double, 20, 2>::Zero();
	std::ofstream outfile; // log the "generate path data".
	outfile.open("gPData.txt", std::ios::trunc | std::ios::out);

	// make sure that the desiredPath matches the cartIniPos and the tarPos.
	if (iniPosIn(0) != desiredPath(0, 0) || iniPosIn(1) != desiredPath(0, 1) || tarPosIn(0) != desiredPath(19, 0) || tarPosIn(1) != desiredPath(19, 1))
	{
		std::cerr << "iniPosIn or tarPosIn doesn't match the desiredPath!" << std::endl;
		return factualPath;
	}
	
	double disThresh = 0.05; // 10mm
	int dpSign = 1; // desiredPath sign post (from 0-19).
	Eigen::Vector2d tarPosi(desiredPath(dpSign,0), desiredPath(dpSign,1));
	Eigen::Vector3d cartPosi(iniPosIn);
	Eigen::Vector2d cartVeli(Eigen::Vector2d::Zero());
	Eigen::Vector2d wheelsVeli(Eigen::Vector2d::Zero());
	Eigen::Vector3d cartVel3i(Eigen::Vector3d::Zero());
	while ((sqrt(pow(cartPosi(0) - tarPosIn(0), 2) + pow(cartPosi(1) - tarPosIn(1), 2)) > disThresh))
	{
		double disErr = sqrt(pow(cartPosi(0) - tarPosi(0), 2) + pow(cartPosi(1) - tarPosi(1), 2));
		if (dpSign < 19)
		{
			double preErr = sqrt(pow(cartPosi(0) - desiredPath(dpSign - 1, 0), 2) + pow(cartPosi(1) - desiredPath(dpSign - 1, 1), 2));
			if (disErr<preErr)
			{
				dpSign++;
				tarPosi << desiredPath(dpSign, 0), desiredPath(dpSign, 1); // update tarPosi.
				disErr = sqrt(pow(cartPosi(0) - tarPosi(0), 2) + pow(cartPosi(1) - tarPosi(1), 2));
			}
		}
		double angErr = atan2(tarPosi(1) - cartPosi(1), tarPosi(0) - cartPosi(0)) - cartPosi(2);
		if (angErr > EIGEN_PI)
			angErr -= 2 * EIGEN_PI;
		else if (angErr < -EIGEN_PI)
			angErr += 2 * EIGEN_PI;

		// compute next position and orientation from information at time i.
		cartVel3i = fromWheelsVel2CartVel3(cartPosi, wheelsVeli);
		Eigen::Vector3d cartPosNi;
		cartPosNi << cartVel3i(0) / cartPara.communFreq + cartPosi(0),
			cartVel3i(1) / cartPara.communFreq + cartPosi(1),
			cartVel3i(2) / cartPara.communFreq + cartPosi(2);
		cartPosi = cartPosNi;

		// comute next velocity.
		// kp is ignored here. (kp=1)
		Eigen::Vector2d cartVelNi(std::min(disErr, cartPara.linVelLim),
			abs(angErr)>cartPara.angVelLim ? cartPara.angVelLim*angErr / abs(angErr) : angErr);
		if ((cartVelNi(0) - cartVeli(0)) * cartPara.communFreq > cartPara.linAccLim)
			cartVelNi(0) = cartVeli(0) + cartPara.linAccLim / cartPara.communFreq;
		else if ((cartVelNi(0) - cartVeli(0)) * cartPara.communFreq < -cartPara.linAccLim)
			cartVelNi(0) = cartVeli(0) - cartPara.linAccLim / cartPara.communFreq;
		if ((cartVelNi(1) - cartVeli(1)) * cartPara.communFreq > cartPara.angAccLim)
			cartVelNi(1) = cartVeli(1) + cartPara.angAccLim / cartPara.communFreq;
		else if ((cartVelNi(1) - cartVeli(1)) * cartPara.communFreq < -cartPara.angAccLim)
			cartVelNi(1) = cartVeli(1) - cartPara.angAccLim / cartPara.communFreq;
		cartVeli = cartVelNi;
		wheelsVeli = fromCartVel2WheelsVel2(cartVeli);
		
		outfile << cartPosi(0) << '\t' << cartPosi(1) << '\t' << cartPosi(2)
			<< '\t' << cartVeli(0) << '\t' << cartVeli(1) << '\n';
	}
	outfile.close();
	return factualPath;
}




