#include "Detector.h"


float focalRange[3] = { 47, 47.6, 48.2 }; // applied voltage, used for find aruco.
float coarseStepSize = 0.1; // coarse search step
float fineStepSize = 0.05; // fine search step
int maxFalseCount = 5; // used in bilateralSearch to determine whether trackAruco1 successfully.

Detector::Detector()
	: accurateFocalVoltage(47)
{
	capturedImg.create(2048, 2448, CV_8UC1);
	aruco1 = ArucoContainer();

	initStatus = true;

	// open caspian
	casp.comConnect();
	if (!casp.getComStatus()) {
		std::cout << "failed to open caspian..." << std::endl;
		initStatus = false;
	}

	// open camera
	if (!GxHandler.openDevice()) {
		std::cout << "failed to open camera..." << std::endl;
		initStatus = false;
	}
	GxHandler.startSnap(); // GxHandler.imgFlow starts to update.

}

Detector::~Detector()
{
	// close caspian
	casp.comClose();

	// close camera
	GxHandler.stopSnap();
	GxHandler.closeDevice();
}

bool Detector::coarseToFine()
{
	// find aruco, update approx focal voltage.
	for (const float& setFocalVoltage : focalRange) {
		casp.setFocusNum(setFocalVoltage);
		updateCapturedImg(GxHandler.getImgFlow());

		cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);	
		std::vector<std::vector<cv::Point2f>> markersCorners;
		std::vector<int> markersId;
		cv::aruco::detectMarkers(capturedImg, dictionary, markersCorners, markersId);

		if (markersId.size() > 0) { // if find marker

			// aruco with maximum area is set marker1
			float maxArea = 0;
			int maxId = -1;
			for (int marker_i = 0; marker_i < markersId.size(); marker_i++) {
				ArucoContainer arucoTemp(markersId.at(marker_i), markersCorners.at(marker_i));
				if (arucoTemp.area.area() > maxArea) {
					maxArea = arucoTemp.area.area();
					maxId = marker_i;
				}
			}
			aruco1 = ArucoContainer(markersId.at(maxId), markersCorners.at(maxId));

			break;
		} 
	}

	if (aruco1.id == -1) { // not find
		return false;
	}

	// coarse bilateral search, coarseStepSize 0.1v
	float approxFocalVoltage = getApproxFV(); // estimate from aruco1 size
	float bestFocalVoltage;
	bool searchSuccess = false;
	searchSuccess = bilateralSearch(approxFocalVoltage, coarseStepSize, bestFocalVoltage);
	if (searchSuccess) {
		updateAccurateFV(bestFocalVoltage);
	}

	return searchSuccess;
}

bool Detector::trackNextCaptured() {

	// fine bilateral search, fineStepSize 0.05v
	float bestFocalVoltage;
	bool searchSuccess = false;
	searchSuccess = bilateralSearch(accurateFocalVoltage, fineStepSize, bestFocalVoltage);
	if (searchSuccess) {
		updateAccurateFV(bestFocalVoltage);
	}

	return searchSuccess;
}

float Detector::getApproxFV() const
{
	int areaSize = aruco1.area.area();
	float approxFocalVoltage = 47.6; // TODO: a model, used to estimate focal voltage.
	return approxFocalVoltage;
}

// bestFocalVoltage is the best focal voltage in current step size
bool Detector::bilateralSearch(float currentFocalVoltage, float stepSize, float& bestFocalVoltage)
{
	int searchDirection = 0;
	float bestSharpness = 0;

	// determine the search direction
	float sharpnessLMH[3]; // low, middle and high.
	float focalVoltage[3] = { currentFocalVoltage - stepSize, currentFocalVoltage, currentFocalVoltage + stepSize };
	for (int i = 0; i < 3; i++) {
		bool trackSuccess = false;
		float fv = focalVoltage[i];
		casp.setFocusNum(fv);

		int falseCount = 0;
		while (!trackSuccess && falseCount < maxFalseCount) { // falseCount > 5, means track failed
			updateCapturedImg(GxHandler.getImgFlow());
			trackSuccess = trackAruco1();
			if (!trackSuccess) {
				falseCount++;
				continue;
			}
			sharpnessLMH[i] = detectSharpness();

		}
		if (falseCount >= maxFalseCount) {
			std::cout << "bilateralSearch - trackAruco1 failed1!" << std::endl;
			return false;
		}
			
	}

	if (sharpnessLMH[1] >= sharpnessLMH[0] && sharpnessLMH[1] >= sharpnessLMH[2]) {
		bestFocalVoltage = focalVoltage[1];
		//if (sharpnessLMH[0] >= sharpnessLMH[2]) {
		//	focalVoltage2 = focalVoltage[0];
		//}
		//else {
		//	focalVoltage2 = focalVoltage[2];
		//}
		return true;
	}
	else if (sharpnessLMH[0] >= sharpnessLMH[2]) {
		searchDirection = -1;
		currentFocalVoltage = focalVoltage[0];
		bestSharpness = sharpnessLMH[0];
	}
	else {
		searchDirection = 1;
		currentFocalVoltage = focalVoltage[2];
		bestSharpness = sharpnessLMH[2];
	}

	// find the best focal voltage
	while (true) {
		int falseCount = 0;
		bool trackSuccess = false;
		float currentSharpness = 0;
		float fv = currentFocalVoltage + stepSize * searchDirection;
		casp.setFocusNum(fv);
		while (!trackSuccess || falseCount < maxFalseCount) { // falseCount > maxFalseCount, means track failed
			updateCapturedImg(GxHandler.getImgFlow());
			trackSuccess = trackAruco1();
			if (!trackSuccess) {
				falseCount++;
				continue;
			}
			currentSharpness = detectSharpness();
		}
		if (falseCount >= maxFalseCount) {
			std::cout << "bilateralSearch - trackAruco1 failed2!" << std::endl;
			return false;
		}

		if (currentSharpness > bestSharpness) {
			bestSharpness = currentSharpness;
			currentFocalVoltage = fv;
		}
		else {
			casp.setFocusNum(currentFocalVoltage);
			bestFocalVoltage = currentFocalVoltage;
			break;
		}
	}

	return true;
}

bool Detector::trackAruco1()
{
	// use last updated aruco1 information to track aruco1
	bool trackSuccess=false;
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

	cv::Point2i center = aruco1.area.tl()+ aruco1.area.br();
	int center_x = center.x / 2;
	int center_y = center.y / 2;
	cv::Rect roiRect;
	for (int iter = 1; iter < 5; iter++) {
		int halfWidth = std::min({ aruco1.area.width * iter, capturedImg.cols - center_x, center_x });
		int halfHeight = std::min({ aruco1.area.height * iter, capturedImg.rows - center_y, center_y });
		roiRect = cv::Rect(center_x - halfWidth, center_y - halfHeight, halfWidth * 2, halfHeight * 2);

		std::vector<std::vector<cv::Point2f>> markersCorners;
		std::vector<int> markersId;
		cv::Mat imgROI(capturedImg, roiRect);
		cv::aruco::detectMarkers(imgROI, dictionary, markersCorners, markersId);
		
		if (markersId.size() > 0) { // find aruco1
			aruco1 = ArucoContainer(markersId.at(0), markersCorners.at(0));
			trackSuccess = true;
			break;
		}

		if (roiRect.area() > (3 / 4 * capturedImg.cols * capturedImg.rows)) {
			break;
		}
	}
	
	return trackSuccess;
}

float Detector::detectSharpness() const
{
	float sharpness = 0;

	cv::Mat ROI(capturedImg, aruco1.area);
	//cv::resize(ROI, ROI, cv::Size(100, 100)); // TODO: Is ROI resize encessary?

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
	for (int i = 0; i < histBinNum; i++) // i represents gradient
	{
		float bin_val = hist.at<float>(i);
		if (i > 10) {
			sharpness += bin_val * i;
			pixelNum += bin_val;
		}
		
	}
	sharpness = sharpness / pixelNum;

	return sharpness;
}





