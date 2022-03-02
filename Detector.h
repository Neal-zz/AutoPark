#pragma once

#include "ComCaspTest.h"
#include "GxCamTest.h"

#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

typedef struct ArucoContainer
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

		maxy = ceil(std::max({ corners.at(0).y, corners.at(1).y, corners.at(2).y, corners.at(3).y }));
		miny = floor(std::min({ corners.at(0).y, corners.at(1).y, corners.at(2).y, corners.at(3).y }));
		maxx = ceil(std::max({ corners.at(0).x, corners.at(1).x, corners.at(2).x, corners.at(3).x }));
		minx = floor(std::min({ corners.at(0).x, corners.at(1).x, corners.at(2).x, corners.at(3).x }));

		area = cv::Rect(minx, miny, maxx - minx + 1, maxy - miny + 1);
	}

	int id; // aruco id
	cv::Rect area; // aruco area. Double area = ROI search region.
	std::vector<cv::Point2f> corners; // aruco corners
}ArucoContainer;

class Detector
{
public:
	Detector();
	~Detector();

	bool coarseToFine();
	bool trackNextCaptured();

	bool getInitStatus() const { return initStatus; };

private:
	void updateCapturedImg(const cv::Mat& imgFlow) { capturedImg = imgFlow; };
	void updateAccurateFV(float bestFocalVoltage) { accurateFocalVoltage = bestFocalVoltage; };
	float getApproxFV() const;
	bool bilateralSearch(float currentFocalVoltage, float stepSize, float& bestFocalVoltge);
	bool trackAruco1();
	float detectSharpness() const;

	bool initStatus; // whether camera and caspian opened successfully?
	ComCaspTest casp; // caspian handler
	GxCamTest GxHandler; // camera handler

	cv::Mat capturedImg;

	ArucoContainer aruco1;

	float accurateFocalVoltage;
	
	// pose
};















