#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <vector>
#include <array>


using PixelType = float;
const int MatType = CV_32FC1; // image format, don't change randomly.

struct Maxima
{
	Maxima(int i, int j, PixelType val)
		: corner(i, j), val(val)
	{}

	cv::Point corner;
	PixelType val;
};
using Maximas = std::vector<Maxima>;

using Corner = cv::Point_<PixelType>;
using Corners = std::vector<Corner>;
struct CornerTemplate
{
	CornerTemplate()
		: point(cv::Point_<PixelType>(0,0)), width(0.0), angle(0), corr(0)
	{}

	CornerTemplate(const Corner& point, int width)
		: point(point), width(width), angle(0), corr(0)
	{}

	Corner point;
	PixelType width;
	PixelType angle;
	PixelType corr;
};
using CornersTemplate = std::vector<CornerTemplate>;

struct DetectRectangle
{
	bool is_full(const cv::Size& size) const
	{
		if (range_x.start == 0 && range_x.end == size.width &&
			range_y.start == 0 && range_y.end == size.height)
		{
			return true;
		}
		return false;
	}

	cv::Range range_x;
	cv::Range range_y;
};

struct QRTemplate
{
	QRTemplate()
		: corner0(cv::Point_<PixelType>(0, 0)), corner1(cv::Point_<PixelType>(0, 0))
		, corner2(cv::Point_<PixelType>(0, 0)), corner3(cv::Point_<PixelType>(0, 0))
		, id(-1)
	{}

	Corner corner0;
	Corner corner1;
	Corner corner2;
	Corner corner3;
	int id; // 0-15 is valid.
};
using QRsTemplate = std::vector<QRTemplate>;

class Detector
{
public:
	Detector(const cv::Size& size);

	QRsTemplate process(const cv::Mat& image);

	void showResult(const cv::String& window_name, const QRsTemplate& QRs, const cv::Mat& image);

private:
	bool detectCorners(const cv::Mat& image, QRsTemplate& QRs_selected);

	void secondDerivCornerMetric(cv::Mat& I_angle, cv::Mat& I_weight, cv::Mat& cmax);
	Maximas nonMaximumSuppression(const cv::Mat& img, int n = 8, int margin = 8, PixelType tau = 0.2f); // tau is the threshold.
	bool detectCornersOnMarker(const Maximas& corners, QRsTemplate& QRs_selected);
	Corner subPixelLocation(const cv::Point& point);
	bool findFirstSecondCorners(const cv::Point& point, CornerTemplate& corner_first, CornerTemplate& corner_second);
	void findEdgeAngles(const Corner& point, PixelType& angle1, PixelType& angle2);
	void edgeOrientation(const cv::Mat& img_angle, const cv::Mat& img_weight, PixelType& angle1, PixelType& angle2);
	PixelType calcBolicCorrelation(const Corner& point, const int& width, const PixelType& theta) const;
	Corner findNextCorner(const CornerTemplate& current, const int& dir, const PixelType& searchAngle);
	//CornerTemplate predictPerpNextCorner(const CornerTemplate& current, const int& dir);
	CornerTemplate predictNextCorner(const CornerTemplate& current, const int& dir);
	int getQRID(const QRTemplate& QR);

	Eigen::MatrixXf calcPatchX();
	cv::Mat conv2(const cv::Mat& img, const cv::Mat& kernel, const cv::String& mode);

	const cv::Size SIZE;  // TODO
	const int SIGMA; // gaussian blur sigma and kernel size = 7*sigma + 1
	const int HALF_PATCH_SIZE; // is used for subPixelLocation.
	const Eigen::MatrixXf PATCH_X; // is used for subPixelLocation.
	const int WIDTH_MIN; // is used for nonMaximumSuppression. 
	const PixelType CORR_THRESHOLD; // 棋盘格角点配对模板时，用到的相关系数阈值。

	DetectRectangle rect; // TODO
	cv::Mat gray_image; // input image.
	cv::Mat I_angle; // gradient direction.
	cv::Mat I_weight; // gradient magnitude.
	cv::Mat cmax; // corners score.
	int initWidth; // is used for initializing searching step. 第一次真正更新，是在 findFirstSecondCorner() 函数中。

	CornersTemplate corners_on_marker;// temp, test use.
};