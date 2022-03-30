#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

struct pose3d
{
	cv::Vec3d rvec;
	cv::Vec3d tvec;
};

extern float markerSize;

extern cv::Mat cameraMatrix;
extern cv::Mat distCoeffs;

extern std::vector<int> estimateIds;
extern std::vector<Eigen::Matrix4f> poseRelativeX;
//extern std::vector<Eigen::Matrix4f> poseRelativeX2;

bool loadConfig();