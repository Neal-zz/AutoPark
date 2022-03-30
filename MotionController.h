#pragma once
#include "config.h"
#include "ComCaspTest.h"
#include "GxCamTest.h"
#include "Detector.h"

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <fstream>

typedef Eigen::Matrix<double, 20, 2> MatrixPath;

struct CartPara
{
	double wheelsRadius;  // mm
	double wheelsDistance;  // distance between the two differential wheels.
	double linVelLim;  // mm/s
	double linAccLim;  // mm/s^2
	double angVelLim;  // rad/s
	double angAccLim;  // rad/s^2
	double communFreq;  // the communication frequency between the car and the computer.

	double r;
	double l1;
	double w;
	double l2;
};



class MotionController
{
public:
	MotionController(const CartPara& cartParaIn);

	//void forwardControl();
	//void closedLoopControl();
	bool registerTrocar();
	bool registerCollidePlane(const std::vector<cv::Point3f>& planePoints);
	bool registerTrolleyPlane(const std::vector<cv::Point3f>& planePoints);
	bool registerTrolleyPlane(Detector& det);
	void registerCartIniPos(const pose3d& det);

	bool MotionController::pathGenerator();
	std::vector<Eigen::Vector2d> fromDesired2FactualPath();  // every path is determined by 20 points.

	//Eigen::Vector3d fromMarkerPos2CartPos(const Eigen::Matrix3d& markerPosIn) const;
	//Eigen::Vector2d fromCartVel2WheelsVel2() const;
	Eigen::Vector2d fromCartVel2WheelsVel2(const Eigen::Vector2d& cartCurrVelIn) const;
	//Eigen::Vector3d fromWheelsVel2CartVel3() const;
	Eigen::Vector3d fromWheelsVel2CartVel3(const Eigen::Vector3d& cartCurrPosIn, const Eigen::Vector2d& wheelsVelIn) const;

	void updateTarPos();
	void updateCartCurrPos(const Eigen::Vector3d& cartCurrPosIn) { cartCurrPos = cartCurrPosIn; };
	void updateCartCurrPos(const pose3d& cartCurrPosIn);
	//void updateCartCurrVel(const Eigen::Vector2d& cartCurrVelIn) { cartCurrVel = cartCurrVelIn; };
	//void updateWheelsVel(const Eigen::Vector2d& wheelsVelIn) { wheelsVel = wheelsVelIn; };
	//void updateTrocarPose(const pose3d& trocarPoseIn) { trocarPose = trocarPoseIn; };
	//void updateTarPos(const Eigen::Vector3d& tarPosIn) { tarPos = tarPosIn; };
	
	Eigen::Vector3d getTarPos() const { return tarPos; };
	Eigen::Vector3d getCartCurrPos() const { return cartCurrPos; };
	//Eigen::Vector2d getCartCurrVel() const { return cartCurrVel; };
	std::vector<Eigen::Vector2d> getRoadSign() const { return roadSign; };
	
	void updateImg(const std::vector<Eigen::Vector2d>& pathRaw = std::vector<Eigen::Vector2d>());
private:
	

	void updateCollide(const Eigen::Vector4d& collide) { collidePlane = collide; };
	void updateTrolleyPlane(const Eigen::Vector4d& trolley) { trolleyPlane = trolley; };
	void updateProjectM();

	Eigen::Vector4d ransacPlane(const std::vector<cv::Point3f>& pts_3d, float threshold) const;
	Eigen::Vector4d svdPlane(const std::vector<cv::Point3f>& plane_pts) const;

	CartPara cartPara;

	pose3d trocarPose;  // tvec; rvec
	Eigen::Vector3d tarPos;  // (x, y, theta), theta: (-pi, pi)
	Eigen::Vector4d collidePlane;  // bed boundary plane.
	Eigen::Vector4d trolleyPlane;
	Eigen::Matrix<double, 3, 4> projectM;  // project 3d points to trolley plane.
	
	Eigen::Vector3d cartIniPos;  // (x, y, theta) update when new markerPos is detected.
	Eigen::Vector3d cartCurrPos;
	//Eigen::Vector2d cartCurrVel;  // (v, thetaDot).
	//Eigen::Vector2d wheelsVel;  // (vR, vL).
	
	std::vector<Eigen::Vector2d> roadSign;  // 20 points roadSign to follow
	std::vector<cv::Point3f> trolleyPoints;  // trajecotry points.

	cv::Mat trajectoryImg;
};




