#include "Detector.h"


float focalRange[2] = { 45.5, 47.5 }; // applied voltage, search range.
float coarseStepSize = 0.5; // coarse search step
float fineStepSize = 0.1; // fine search step
int maxFalseCount = 3; // used in bilateralSearch to determine whether trackAruco1 successfully.
int coarseSleepTime = 50;
int fineSleepTime = 1; // don't sleep.

Detector::Detector()
	: accurateFocalVoltage((focalRange[0]+ focalRange[1])/2)
{
	capturedImg.create(2048, 2448, CV_8UC1);
	aruco1 = ArucoContainer();

	initStatus = true;

	// open caspian
	casp.comConnect();
	if (!casp.getComStatus()) {
		initStatus = false;
	}

	// open camera
	if (!GxHandler.openDevice()) {
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
	// find max aruco 
	casp.setFocusVoltage((focalRange[0]+focalRange[1])/2);
	Sleep(200); // wait until focus ready.

	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
	std::vector<std::vector<cv::Point2f>> markersCorners;
	std::vector<int> markersId;
	while (markersId.size() == 0) {
		updateCapturedImg(GxHandler.getImgFlow());
		cv::aruco::detectMarkers(capturedImg, dictionary, markersCorners, markersId);
	}
	
	std::cout << "Detector::coarseToFine find marker!" << std::endl;

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

	// coarse bilateral search, coarseStepSize 0.5v
	float bestFocalVoltage;
	float LocalBestFV = (focalRange[0] + focalRange[1]) / 2;
	if (!bilateralSearch(LocalBestFV, coarseStepSize, coarseSleepTime, bestFocalVoltage)) {
		return false;
	}

	std::cout << "Detector::coarseToFine coarse search success!" << std::endl;

	// fine bilateral search, fineStepSize 0.1v
	LocalBestFV = bestFocalVoltage;
	if (!bilateralSearch(LocalBestFV, fineStepSize, fineSleepTime, bestFocalVoltage)) {
		return false;
	}

	std::cout << "Detector::coarseToFine fine search success!" << std::endl;

	updateAccurateFV(bestFocalVoltage);
	return true;
}

bool Detector::trackNextCaptured() {

	// fine bilateral search, fineStepSize 0.1v
	float bestFocalVoltage;
	if (!bilateralSearch(accurateFocalVoltage, 0.2, fineSleepTime, bestFocalVoltage)) { // 试试看调大步长到 0.2
		return false;
	}

	updateAccurateFV(bestFocalVoltage);
	return true;
}

// bestFocalVoltage is the best focal voltage in current step size
// we assume that the input currentFocalVoltage == casp.getFocusVoltage.
bool Detector::bilateralSearch(float currentFocalVoltage, float stepSize, int sleepTime, float& bestFocalVoltage)
{
	int searchDirection = 0;
	float bestSharpness = 0;

	if (abs(currentFocalVoltage - casp.getFocusVoltage()) > (0.8 * fineStepSize)) { // min step size is 0.1
		std::cout << "something went wrong in Detector::bilateralSearch." << std::endl;
		return false;
	}

	// get sharpness at currentFocalVoltage
	bool trackSuccess = false;
	int falseCount = 0;
	float sharpnessM = 0;
	while (!trackSuccess && falseCount < maxFalseCount) { // falseCount > 3, means track failed
		updateCapturedImg(GxHandler.getImgFlow());
		trackSuccess = trackAruco1();
		if (!trackSuccess) {
			falseCount++;
			continue;
		}
		casp.setFocusVoltage(currentFocalVoltage + stepSize); // set next focus voltage.
		Sleep(sleepTime); // Sleep time may be unnecessary? TODO
		sharpnessM = detectSharpness();
	}
	if (falseCount >= maxFalseCount) {
		std::cout << "Detector::bilateralSearch - trackAruco1 failed1!" << std::endl;
		return false;
	}

	// first search right handside
	trackSuccess = false;
	falseCount = 0;
	float sharpnessH = 0;
	while (!trackSuccess && falseCount < maxFalseCount) { // falseCount > 3, means track failed
		updateCapturedImg(GxHandler.getImgFlow());
		trackSuccess = trackAruco1();
		if (!trackSuccess) {
			falseCount++;
			continue;
		}
		casp.setFocusVoltage(currentFocalVoltage - stepSize); // set next focus voltage.
		Sleep(sleepTime); // Sleep time may be unnecessary? TODO
		sharpnessH = detectSharpness();
	}
	if (falseCount >= maxFalseCount) {
		std::cout << "Detector::bilateralSearch - trackAruco1 failed2!" << std::endl;
		return false;
	}
	
	if (sharpnessH > sharpnessM) {
		searchDirection = 1;
		bestSharpness = sharpnessH;
		currentFocalVoltage += stepSize;
		casp.setFocusVoltage(currentFocalVoltage + stepSize); // set next focus voltage.
		Sleep(sleepTime); // Sleep time may be unnecessary? TODO
	}
	else { // else search left handside
		trackSuccess = false;
		falseCount = 0;
		float sharpnessL = 0;
		while (!trackSuccess && falseCount < maxFalseCount) { // falseCount > 3, means track failed
			updateCapturedImg(GxHandler.getImgFlow());
			trackSuccess = trackAruco1();
			if (!trackSuccess) {
				falseCount++;
				continue;
			}

			sharpnessL = detectSharpness();
		}
		if (falseCount >= maxFalseCount) {
			std::cout << "Detector::bilateralSearch - trackAruco1 failed3!" << std::endl;
			return false;
		}

		if (sharpnessL > sharpnessM) {
			searchDirection = -1;
			bestSharpness = sharpnessL;
			currentFocalVoltage -= stepSize;
			casp.setFocusVoltage(currentFocalVoltage - stepSize); // set next focus voltage.
			Sleep(sleepTime); // Sleep time may be unnecessary? TODO
		}
	}

	// search towards searchDirection, and find the best focal voltage
	if (searchDirection == 0) {
		bestFocalVoltage = currentFocalVoltage;
		casp.setFocusVoltage(currentFocalVoltage); // set next focus voltage.
		Sleep(sleepTime); // Sleep time may be unnecessary? TODO
		return true;
	}

	while (true) {
		falseCount = 0;
		trackSuccess = false;
		float currentSharpness = 0;
		while (!trackSuccess && falseCount < maxFalseCount) { // falseCount > maxFalseCount, means track failed
			updateCapturedImg(GxHandler.getImgFlow());
			trackSuccess = trackAruco1();
			if (!trackSuccess) {
				falseCount++;
				continue;
			}
			casp.setFocusVoltage(currentFocalVoltage + 2 * searchDirection * stepSize); // set next focus voltage.
			Sleep(sleepTime); // Sleep time may be unnecessary? TODO
			currentSharpness = detectSharpness();
		}
		if (falseCount >= maxFalseCount) {
			std::cout << "bilateralSearch - trackAruco1 failed2!" << std::endl;
			return false;
		}

		if (currentSharpness > bestSharpness) {
			bestSharpness = currentSharpness;
			currentFocalVoltage += searchDirection * stepSize;
		}
		else {
			casp.setFocusVoltage(currentFocalVoltage);
			Sleep(sleepTime);
			bestFocalVoltage = currentFocalVoltage;
			break;
		}
	}

	return true;
}

bool Detector::trackAruco1()
{
	// use last updated aruco1 information, double aruco1.area to track aruco1
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
		cv::Mat imgTemp(capturedImg);
		cv::Mat imgROI(capturedImg, roiRect);
		cv::aruco::detectMarkers(imgROI, dictionary, markersCorners, markersId);
		
		if (markersId.size() > 0) { // find aruco1
			std::vector<cv::Point2f > markerCorners;
			markerCorners.emplace_back(cv::Point2f(center_x - halfWidth + markersCorners.at(0).at(0).x,
				center_y - halfHeight + markersCorners.at(0).at(0).y));
			markerCorners.emplace_back(cv::Point2f(center_x - halfWidth + markersCorners.at(0).at(1).x,
				center_y - halfHeight + markersCorners.at(0).at(1).y));
			markerCorners.emplace_back(cv::Point2f(center_x - halfWidth + markersCorners.at(0).at(2).x,
				center_y - halfHeight + markersCorners.at(0).at(2).y));
			markerCorners.emplace_back(cv::Point2f(center_x - halfWidth + markersCorners.at(0).at(3).x,
				center_y - halfHeight + markersCorners.at(0).at(3).y));
			aruco1 = ArucoContainer(markersId.at(0), markerCorners);
			trackSuccess = true;
			break;
		}

		if (roiRect.area() > (3 / 4 * capturedImg.cols * capturedImg.rows)) { // roi too big
			break;
		}
	}
	
	return trackSuccess;
}

float Detector::detectSharpness() const
{
	float sharpness = 0;

	//cv::Mat ROITmp = capturedImg;
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
	for (int i = 95; i < histBinNum; i++) // i represents gradient; 95 = 255/2 * 3/4
	{
		float bin_val = hist.at<float>(i);
		sharpness += bin_val * i;
		pixelNum += bin_val;
	}
	sharpness = sharpness / pixelNum;

	return sharpness;
}





