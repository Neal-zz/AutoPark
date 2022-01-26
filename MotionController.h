#pragma once

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <fstream>

typedef Eigen::Matrix<double, 20, 2> MatrixPath;

struct CartPara
{
	double wheelsRadius;
	double wheelsDistance; // distance between the two differential wheels.
	double linVelLim;
	double linAccLim;
	double angVelLim;
	double angAccLim;
	double communFreq; // the communication frequency between the car and the computer.
};


class MotionController
{
public:
	MotionController(const CartPara& cartParaIn);

	Eigen::Vector3d fromMarkerPos2CartPos(const Eigen::Matrix3d& markerPosIn) const;

	Eigen::Vector2d fromCartVel2WheelsVel2() const;

	Eigen::Vector2d fromCartVel2WheelsVel2(const Eigen::Vector2d& cartCurrVelIn) const;

	Eigen::Vector3d fromWheelsVel2CartVel3() const;

	Eigen::Vector3d fromWheelsVel2CartVel3(const Eigen::Vector3d& cartCurrPosIn, const Eigen::Vector2d& wheelsVelIn) const;

	MatrixPath fromDesired2FactualPath(const MatrixPath& desiredPath, const Eigen::Vector3d& iniPosIn, const Eigen::Vector3d& tarPosIn) const; // every path is determined by 20 points.

	inline void updateCartIniPos(const Eigen::Vector3d& cartIniPosIn) { cartIniPos = cartIniPosIn; cartCurrPos = cartIniPosIn; };

	inline void updateCartCurrPos(const Eigen::Vector3d& cartCurrPosIn) { cartCurrPos = cartCurrPosIn; };

	inline void updateTarPos(const Eigen::Vector3d& tarPosIn) { tarPos = tarPosIn; };

	inline void updateCartCurrVel(const Eigen::Vector2d& cartCurrVelIn) { cartCurrVel = cartCurrVelIn; };
	
	inline void updateWheelsVel(const Eigen::Vector2d& wheelsVelIn) { wheelsVel = wheelsVelIn; };
	
	inline Eigen::Vector3d getCartCurrPos() const { return cartCurrPos; };

	inline Eigen::Vector2d getCartCurrVel() const { return cartCurrVel; };

private:
	CartPara cartPara;
	Eigen::Vector3d cartIniPos; // (x, y, theta) update when new markerPos is detected.
	Eigen::Vector3d cartCurrPos;
	Eigen::Vector3d tarPos; // (x, y, theta)
	Eigen::Vector2d cartCurrVel; // (v, thetaDot).
	Eigen::Vector2d wheelsVel; // (vR, vL).
	//Eigen::Vector3d cartAcc;

};




