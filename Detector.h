#pragma once

#include "config.h"
#include "ComCaspTest.h"
#include "GxCamTest.h"

#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <Eigen/Core>

class MotionController;

struct ArucoContainer
{
	ArucoContainer()
		: id(-1), area(0,0,0,0)
	{
		corners.clear();
	}

	ArucoContainer(int idIn, const std::vector<cv::Point2f>& cornersIn)
	{
		id = idIn;
		corners = cornersIn;
		updateArea();
	}

	void updateArea() {
		int miny, minx, maxy, maxx;
		maxy = ceil((std::max)({ corners.at(0).y, corners.at(1).y, corners.at(2).y, corners.at(3).y }));
		miny = floor((std::min)({ corners.at(0).y, corners.at(1).y, corners.at(2).y, corners.at(3).y }));
		maxx = ceil((std::max)({ corners.at(0).x, corners.at(1).x, corners.at(2).x, corners.at(3).x }));
		minx = floor((std::min)({ corners.at(0).x, corners.at(1).x, corners.at(2).x, corners.at(3).x }));

		area = cv::Rect(minx, miny, maxx - minx + 1, maxy - miny + 1);
	}

	int id; // aruco id
	cv::Rect area; // aruco area. Double area = ROI search region.
	std::vector<cv::Point2f> corners; // aruco corners
};
using ArucoContainers = std::vector<ArucoContainer>;

using Corner = cv::Point_<float>;
using Corners = std::vector<Corner>;

class Detector
{
public:
	Detector(std::shared_ptr<ComCaspTest> caspian, std::shared_ptr<GxCamTest> GxCamera);

	bool coarseToFine(const float& searchFVMin, const float& searchFVMax);
	bool estimatePose(bool firstEstimate = false);

	Corner rectifyOneCorner(const Corner& corner) const; // from image points to true points.
	
	Eigen::Matrix4f getTC_M() const;  // Tcamera_marker.
	pose3d getPose() const { return markerPose; };  // posecamer_marker.
	
	std::vector<cv::Point3f> registerPlane();

	//pose3d detectTrolley();

private:
	void updateCapturedImg(const cv::Mat& imgFlow) { capturedImg = imgFlow; };
	//void updateAccurateFV(float bestFocalVoltage) { accurateFocalVoltage = bestFocalVoltage; };
	//float getApproxFV() const; // get approximate focal voltage
	bool bilateralSearch(float currentFocalVoltage, float stepSize, int sleepTime);
	bool trackAruco1();
	float detectSharpness() const;
	cv::Rect getSearchRegion() const;
	Corners getReprojectImagePoint(const std::vector<cv::Point3f>& object3DPoints) const;
	ArucoContainers subPixelCorners(const ArucoContainers& intACs) const;
	void update_rt(const cv::Vec3d& rvecTemp, const cv::Vec3d& tvecTemp) { markerPose.rvec = rvecTemp; markerPose.tvec = tvecTemp; };
	float fromDistance2FV() const;
	void showResult(const std::vector<cv::Point3f>& pointsM, const std::vector<cv::Point2f>& pointsP) const;
	void saveImage(const cv::Mat& imagem, const std::string& strPrefix = "") const;
	void drawArucoDetectedMarkers(cv::Mat& ROI, const ArucoContainers& subpixelMarkers) const;

	std::shared_ptr<ComCaspTest> caspHandler; // caspian handler
	std::shared_ptr<GxCamTest> GxHandler; // camera handler

	std::string strFilePath;

	cv::Mat capturedImg;

	ArucoContainer aruco1;
	//ArucoContainers arucos;

	//float accurateFocalVoltage; // current best Focal Voltage, used for trackNextCaptured().

	// pose, Tcam_Marker
	pose3d markerPose;
	//cv::Vec3d rvec;
	//cv::Vec3d tvec;

	std::mutex tT_mutex;
	std::vector<cv::Point3f> trolleyTrajectory;
	friend class MotionController;
};





