#include "Detector.h"


float focalRange[2] = { 48.9, 50.9 }; // applied voltage, search range for trolley.
//float coarseStepSize = 0.5; // coarse search step.
float fineStepSize = 0.1; // fine search step.
int maxFalseCount = 3; // used in bilateralSearch to determine whether trackAruco1 successfully.
int coarseSleepTime = 400;
int fineSleepTime = 200;
int imgWidth = 2448;
int imgHeight = 2048;

Detector::Detector(std::shared_ptr<ComCaspTest> caspian, std::shared_ptr<GxCamTest> GxCamera) //	: accurateFocalVoltage((focalRange[0]+ focalRange[1])/2)
	: strFilePath("E:\\sjtu\\autoPark\\code\\autoPark\\autoPark\\poseEstimate\\")
	, aruco1(ArucoContainer())
	, caspHandler(caspian)
	, GxHandler(GxCamera)
{
	markerPose.rvec = { 0.0,0.0,0.0 };
	markerPose.tvec = { 0.0,0.0,0.0 };

	capturedImg.create(imgHeight, imgWidth, CV_8UC1);
	//capturedImg = cv::imread("E:\\sjtu\\autoPark\\code\\calibrate\\accuracyTest\\4m\\15.bmp", CV_8UC1);

}

bool Detector::coarseToFine(const float& searchFVMin, const float& searchFVMax) {
	
	// set FV = middle value.
	int localBestFVTemp = static_cast<int>((searchFVMin + searchFVMax) * 5);
	float localBestFV = static_cast<int>(localBestFVTemp) / 10.0;
	caspHandler->setFocusVoltage(localBestFV);
	std::this_thread::sleep_for(std::chrono::milliseconds(coarseSleepTime));  // wait until focus ready.
	
	// find max aruco 
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
	std::vector<std::vector<cv::Point2f>> markersCorners;
	std::vector<int> markersId;
	while (markersId.size() == 0) {
		updateCapturedImg(GxHandler->getImgFlow());
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

	// fine bilateral search, fineStepSize 0.1v
	if (!bilateralSearch(localBestFV, fineStepSize, fineSleepTime)) {
		return false;
	}
	std::cout << "Detector::coarseToFine search success!" << std::endl;

	return true;
}

bool Detector::estimatePose(bool firstEstimate) {
	updateCapturedImg(GxHandler->getImgFlow());

	// get search region and resize scale.
	// detect markers.
	cv::Rect searchRegion;
	float resizeScale;
	cv::Mat ROI;
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
	std::vector<std::vector<cv::Point2f>> markersCornersRaw;
	cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
	if (firstEstimate == true) {
		searchRegion = cv::Rect(0, 0, imgWidth, imgHeight);
		resizeScale = 1.0;
		ROI = capturedImg.clone();
		parameters->minMarkerPerimeterRate = 0.1;
		parameters->maxMarkerPerimeterRate = 0.8;
	}
	else {
		searchRegion = getSearchRegion();
		resizeScale = ceil(sqrt(150000.0 / searchRegion.area()) * 10.0) / 10.0; // 600 * 250
		ROI = capturedImg(searchRegion);
		cv::resize(ROI, ROI, cv::Size(), resizeScale, resizeScale);
		parameters->minMarkerPerimeterRate = 0.2; // the minimum pixel size of a marker, specified relative to the maximum dimension of the input image.
		parameters->maxMarkerPerimeterRate = 2; // the minimum pixel size of a marker... // 0.8;
		parameters->minMarkerDistanceRate = 0.01; // Minimum distance between any pair of corners from two different markers.
	}
	std::vector<int> markersIdRaw;
	cv::aruco::detectMarkers(ROI, dictionary, markersCornersRaw, markersIdRaw, parameters);

	// if at least one marker detected
	if (markersIdRaw.size() == 0) {
		std::cout << "estimatePose() no marker..." << std::endl;
		//saveImage(ROI, "roi");
		return false;
	}

	// two boundaries condition. Delete the outer marker.
	std::vector<std::vector<cv::Point2f>> markersCornersScaled;
	std::vector<int> markersId;
	auto first = markersIdRaw.begin();
	auto ff = markersCornersRaw.begin();
	while (first != markersIdRaw.end()) {
		bool unique = true;
		auto second = first + 1;
		auto ss = ff + 1;
		while (second != markersIdRaw.end()) {
			if (*second == *first) {
				unique = false;
				if ((fabs(ff->at(0).x - ff->at(1).x) + fabs(ff->at(0).y - ff->at(3).y)) > (fabs(ss->at(0).x - ss->at(1).x) + fabs(ss->at(0).y - ss->at(3).y))) {
					markersId.emplace_back(*second);
					markersCornersScaled.emplace_back(*ss);
				}
				else {
					markersId.emplace_back(*first);
					markersCornersScaled.emplace_back(*ff);
				}
				markersIdRaw.erase(second);
				markersCornersRaw.erase(ss);
				break;
			}
			second++;
			ss++;
		}
		if (unique) {
			markersId.emplace_back(*first);
			markersCornersScaled.emplace_back(*ff);
		}
		first++;
		ff++;
	}
	cv::aruco::drawDetectedMarkers(ROI, markersCornersScaled, markersId); // draw the results.
	//saveImage(ROI, "roi");
	ArucoContainers markerCorners;
	int mCSi = 0;
	for (const std::vector<cv::Point2f>& mCS : markersCornersScaled) {
		std::vector<cv::Point2f > mC;
		mC.emplace_back((mCS.at(0).x + 1.0) / resizeScale - 1.0 + static_cast<float>(searchRegion.x), (mCS.at(0).y + 1.0) / resizeScale - 1.0 + static_cast<float>(searchRegion.y));
		mC.emplace_back((mCS.at(1).x + 1.0) / resizeScale - 1.0 + static_cast<float>(searchRegion.x), (mCS.at(1).y + 1.0) / resizeScale - 1.0 + static_cast<float>(searchRegion.y));
		mC.emplace_back((mCS.at(2).x + 1.0) / resizeScale - 1.0 + static_cast<float>(searchRegion.x), (mCS.at(2).y + 1.0) / resizeScale - 1.0 + static_cast<float>(searchRegion.y));
		mC.emplace_back((mCS.at(3).x + 1.0) / resizeScale - 1.0 + static_cast<float>(searchRegion.x), (mCS.at(3).y + 1.0) / resizeScale - 1.0 + static_cast<float>(searchRegion.y));
		ArucoContainer mCTemp(markersId[mCSi], mC);
		markerCorners.emplace_back(mCTemp);
		mCSi++;
	}

	// subpixel location
	ArucoContainers subpixelMarkers=subPixelCorners(markerCorners);
	//cv::Mat temp_image = capturedImg;

	// pose estimate
	std::vector<cv::Point3f> pointsM;
	std::vector<cv::Point2f> pointsP;
	for (int i = 0; i < estimateIds.size(); i++) {
		std::vector<int>::iterator iter = std::find(markersId.begin(), markersId.end(), estimateIds[i]);
		if (iter != markersId.end()) {
			int idNumTemp = std::distance(markersId.begin(), iter);

			Eigen::Vector4f pointX_0(-markerSize / 2, markerSize / 2, 0.0, 1.0);
			Eigen::Vector4f pointX_1(markerSize / 2, markerSize / 2, 0.0, 1.0);
			Eigen::Vector4f pointX_2(markerSize / 2, -markerSize / 2, 0.0, 1.0);
			Eigen::Vector4f pointX_3(-markerSize / 2, -markerSize / 2, 0.0, 1.0);
			Eigen::Vector4f pointM_0 = poseRelativeX[i] * pointX_0;
			Eigen::Vector4f pointM_1 = poseRelativeX[i] * pointX_1;
			Eigen::Vector4f pointM_2 = poseRelativeX[i] * pointX_2;
			Eigen::Vector4f pointM_3 = poseRelativeX[i] * pointX_3;
			pointsM.emplace_back(pointM_0(0), pointM_0(1), pointM_0(2));
			pointsM.emplace_back(pointM_1(0), pointM_1(1), pointM_1(2));
			pointsM.emplace_back(pointM_2(0), pointM_2(1), pointM_2(2));
			pointsM.emplace_back(pointM_3(0), pointM_3(1), pointM_3(2));

			pointsP.emplace_back(subpixelMarkers.at(idNumTemp).corners.at(0));
			pointsP.emplace_back(subpixelMarkers.at(idNumTemp).corners.at(1));
			pointsP.emplace_back(subpixelMarkers.at(idNumTemp).corners.at(2));
			pointsP.emplace_back(subpixelMarkers.at(idNumTemp).corners.at(3));
		}
	}
	cv::Vec3d rvecTemp, tvecTemp;
	if (pointsP.size() > 4) {
		cv::solvePnP(pointsM, pointsP, cameraMatrix, distCoeffs, rvecTemp, tvecTemp, false, cv::SOLVEPNP_EPNP); // r and t: Tcam_M
	}
	else {
		cv::solvePnP(pointsM, pointsP, cameraMatrix, distCoeffs, rvecTemp, tvecTemp, false, cv::SOLVEPNP_P3P); // r and t: Tcam_M
	}
	update_rt(rvecTemp, tvecTemp);
	showResult(pointsM, pointsP); // draw results

	// update focal voltage and camera parameters.
	/*float currentBestFV = fromDistance2FV();
	casp.setFocusVoltage(currentBestFV);*/

	return true;
}

// bestFocalVoltage is the best focal voltage in current step size
// we assume that the input currentFocalVoltage == casp.getFocusVoltage.
bool Detector::bilateralSearch(float currentFocalVoltage, float stepSize, int sleepTime)
{
	int searchDirection = 0;
	float bestSharpness = 0;

	if (fabs(currentFocalVoltage - caspHandler->getFocusVoltage()) > (0.8 * fineStepSize)) {  // min step size is 0.1
		std::cout << "something went wrong in Detector::bilateralSearch()." << std::endl;
		return false;
	}

	// get sharpness at currentFocalVoltage
	bool trackSuccess = false;
	int falseCount = 0;
	float sharpnessM = 0;
	while (!trackSuccess && falseCount < maxFalseCount) {  // falseCount > 3, means track failed
		updateCapturedImg(GxHandler->getImgFlow());
		trackSuccess = trackAruco1();
		if (!trackSuccess) {
			falseCount++;
			continue;
		}
		caspHandler->setFocusVoltage(currentFocalVoltage + stepSize);  // set next focus voltage.
		std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));  // Sleep time may be unnecessary? TODO
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
		updateCapturedImg(GxHandler->getImgFlow());
		trackSuccess = trackAruco1();
		if (!trackSuccess) {
			falseCount++;
			continue;
		}
		caspHandler->setFocusVoltage(currentFocalVoltage - stepSize); // set next focus voltage.
		std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));  // Sleep time may be unnecessary? TODO
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
		caspHandler->setFocusVoltage(currentFocalVoltage + stepSize); // set next focus voltage.
		std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));  // Sleep time may be unnecessary? TODO
	}
	else { // else search left handside
		trackSuccess = false;
		falseCount = 0;
		float sharpnessL = 0;
		while (!trackSuccess && falseCount < maxFalseCount) { // falseCount > 3, means track failed
			updateCapturedImg(GxHandler->getImgFlow());
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
			caspHandler->setFocusVoltage(currentFocalVoltage - stepSize); // set next focus voltage.
			std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));  // Sleep time may be unnecessary? TODO
		}
	}

	// search towards searchDirection, and find the best focal voltage
	if (searchDirection == 0) {
		//bestFocalVoltage = currentFocalVoltage;
		caspHandler->setFocusVoltage(currentFocalVoltage); // set next focus voltage.
		std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));  // Sleep time may be unnecessary? TODO
		return true;
	}

	while (true) {
		falseCount = 0;
		trackSuccess = false;
		float currentSharpness = 0;
		while (!trackSuccess && falseCount < maxFalseCount) { // falseCount > maxFalseCount, means track failed
			updateCapturedImg(GxHandler->getImgFlow());
			trackSuccess = trackAruco1();
			if (!trackSuccess) {
				falseCount++;
				continue;
			}
			caspHandler->setFocusVoltage(currentFocalVoltage + 2 * searchDirection * stepSize); // set next focus voltage.
			std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));  // Sleep time may be unnecessary? TODO
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
			caspHandler->setFocusVoltage(currentFocalVoltage);
			std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));  // Sleep time may be unnecessary? TODO
			//bestFocalVoltage = currentFocalVoltage;
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
		int halfWidth = (std::min)({ aruco1.area.width * iter, capturedImg.cols - center_x, center_x });
		int halfHeight = (std::min)({ aruco1.area.height * iter, capturedImg.rows - center_y, center_y });
		roiRect = cv::Rect(center_x - halfWidth, center_y - halfHeight, halfWidth * 2, halfHeight * 2);

		std::vector<std::vector<cv::Point2f>> markersCorners;
		std::vector<int> markersId;
		//cv::Mat imgTemp(capturedImg); // test
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

	// two tyes of sharpness detect algorithm.
	int pixelNum = 0;
	for (int i = 20; i < histBinNum; i++) // i represents gradient; 95 = 255/2 * 3/4
	{
		float bin_val = hist.at<float>(i);
		sharpness += bin_val *i;
		pixelNum += bin_val;
	}
	//for (int i = histBinNum - 1; i > 0; i--) {
	//	float bin_val = hist.at<float>(i);
	//	sharpness += bin_val * i;
	//	pixelNum += bin_val;
	//	if (pixelNum > 500) {
	//		break;
	//	}
	//}
	sharpness = sharpness / static_cast<float>(pixelNum);

	return sharpness;
}

Corner Detector::rectifyOneCorner(const Corner& corner_) const {
	Corner cornerRec;

	float x_uv, y_uv;
	x_uv = (corner_.x - cameraMatrix.at<float>(0, 2)) / cameraMatrix.at<float>(0, 0);
	y_uv = (corner_.y - cameraMatrix.at<float>(1, 2)) / cameraMatrix.at<float>(1, 1);
	Corner pixel_ud = Corner(x_uv, y_uv);

	float r_xy = pixel_ud.x * pixel_ud.x + pixel_ud.y * pixel_ud.y;
	float k1 = distCoeffs.at<float>(0,0);
	float k2 = distCoeffs.at<float>(1,0);
	float p1 = distCoeffs.at<float>(2,0);
	float p2 = distCoeffs.at<float>(3,0);
	float x_d = pixel_ud.x * (1 + k1 * r_xy + k2 * r_xy * r_xy) + 2 * p1 * pixel_ud.x * pixel_ud.y + p2 * (r_xy + 2 * pixel_ud.x * pixel_ud.x);
	float y_d = pixel_ud.y * (1 + k1 * r_xy + k2 * r_xy * r_xy) + 2 * p2 * pixel_ud.x * pixel_ud.y + p1 * (r_xy + 2 * pixel_ud.y * pixel_ud.y);

	cornerRec.x = cameraMatrix.at<float>(0, 0) * x_d + cameraMatrix.at<float>(0, 2);
	cornerRec.y = cameraMatrix.at<float>(1, 1) * y_d + cameraMatrix.at<float>(1, 2);
	
	return cornerRec;
}


cv::Rect Detector::getSearchRegion() const {

	std::vector<cv::Point3f> pointsM;
	for (const Eigen::Matrix4f& poseM_X : poseRelativeX) {
		Eigen::Vector4f pointX_0(-markerSize / 2, markerSize / 2, 0.0, 1.0);
		Eigen::Vector4f pointX_1(markerSize / 2, markerSize / 2, 0.0, 1.0);
		Eigen::Vector4f pointX_2(markerSize / 2, -markerSize / 2, 0.0, 1.0);
		Eigen::Vector4f pointX_3(-markerSize / 2, -markerSize / 2, 0.0, 1.0);
		Eigen::Vector4f pointM_0 = poseM_X * pointX_0;
		Eigen::Vector4f pointM_1 = poseM_X * pointX_1;
		Eigen::Vector4f pointM_2 = poseM_X * pointX_2;
		Eigen::Vector4f pointM_3 = poseM_X * pointX_3;
		pointsM.emplace_back(pointM_0(0), pointM_0(1), pointM_0(2));
		pointsM.emplace_back(pointM_1(0), pointM_1(1), pointM_1(2));
		pointsM.emplace_back(pointM_2(0), pointM_2(1), pointM_2(2));
		pointsM.emplace_back(pointM_3(0), pointM_3(1), pointM_3(2));
	}
	Corners reproPoints;
	reproPoints = getReprojectImagePoint(pointsM);

	float max_x = 0;
	float max_y = 0;
	float min_x = imgWidth;
	float min_y = imgHeight;
	for (const Corner& rp : reproPoints) {
		if (rp.x < min_x) {
			min_x = rp.x;
		}

		if (rp.x > max_x) {
			max_x = rp.x;
		}

		if (rp.y < min_y) {
			min_y = rp.y;
		}

		if (rp.y > max_y) {
			max_y = rp.y;
		}
	}

	int width = floor(max_x - min_x);
	int height = floor(max_y - min_y);
	int init_x = ceil(std::max<float>({ min_x - width / 2, 0.0 }));
	int init_y = ceil(std::max<float>({ min_y - height / 2, 0.0 }));
	cv::Rect searchRegion(init_x, init_y, std::min<int>({ imgWidth - init_x, width * 2 }), std::min<int>({ imgHeight - init_y, height * 2 }));

	return searchRegion;
}

Eigen::Matrix4f Detector::getTC_M() const {
	Eigen::Matrix4f TC_M;
	cv::Mat rotm;
	cv::Rodrigues(markerPose.rvec, rotm);
	TC_M << rotm.at<float>(0, 0), rotm.at<float>(0, 1), rotm.at<float>(0, 2), markerPose.tvec(0),
		rotm.at<float>(1, 0), rotm.at<float>(1, 1), rotm.at<float>(1, 2), markerPose.tvec(1),
		rotm.at<float>(2, 0), rotm.at<float>(2, 1), rotm.at<float>(2, 2), markerPose.tvec(2),
		0.0, 0.0, 0.0, 1.0;
	return TC_M;
}

Corners Detector::getReprojectImagePoint(const std::vector<cv::Point3f>& object3DPoints) const {
	cv::Mat reprojectedPoints;
	cv::InputArray object3DPoints_x = (cv::InputArray)object3DPoints;
	cv::projectPoints(object3DPoints, markerPose.rvec, markerPose.tvec, cameraMatrix, distCoeffs, reprojectedPoints);

	Corners corners_res = Corners();
	size_t n = object3DPoints_x.rows() * object3DPoints_x.cols();
	for (size_t i = 0; i < n; i++)
	{
		corners_res.push_back(Corner(reprojectedPoints.at<cv::Vec2f>(i)[0], reprojectedPoints.at<cv::Vec2f>(i)[1]));
	}

	return corners_res;
}

/*get subpixel corners*/
ArucoContainers Detector::subPixelCorners(const ArucoContainers& intACs) const {
	
	// corner region size.
	int HalfH = 4;
	int HalfW = 4;
	int winH = HalfH * 2 + 1;
	int winW = HalfW * 2 + 1;
	int winCnt = winH * winW;
	// iteration exit criteria
	int max_iters = 5;
	float eps = 0.000001; // 1e-6

	// gaussian weight mask
	cv::Mat weightMask = cv::Mat(winH, winW, CV_32FC1);
	for (int i = 0; i < winH; i++)
	{
		for (int j = 0; j < winW; j++)
		{
			float wx = (float)(j - HalfW) / HalfW;
			float wy = (float)(i - HalfH) / HalfH;
			float vx = exp(-wx * wx);
			float vy = exp(-wy * wy);
			weightMask.at<float>(i, j) = (float)(vx * vy);
		}
	}

	// for all the points
	ArucoContainers floatACs;
	for (const ArucoContainer& intAC : intACs) {
		std::vector<cv::Point2f > mC;

		for (const cv::Point2f& currPoint : intAC.corners) {

			double a, b, c, bb1, bb2;
			cv::Mat subImg = cv::Mat::zeros(winH + 2, winW + 2, CV_8UC1); // +2 for sobel operation.
			cv::Point2f iterPoint = currPoint;

			// start iteration
			int iterCnt = 0;
			float err = 1.0;
			while (err > eps && iterCnt < max_iters) {
				a = b = c = bb1 = bb2 = 0;

				int initx = int(iterPoint.x + 0.5f) - (winW + 1) / 2;
				int inity = int(iterPoint.y + 0.5f) - (winH + 1) / 2;
				if (initx < 0 || inity < 0 || (initx + winW + 2) >= imgWidth || (inity + winH + 2) >= imgHeight) {
					break;
				}
				cv::Rect imgROI(initx, inity, winW + 2, winH + 2);
				subImg = capturedImg(imgROI).clone();

				for (int i = 0; i < winH; i++) {
					for (int j = 0; j < winW; j++) {
						// read gaussian weight.
						double m = weightMask.at<float>(i, j);
						// soble operation.
						double sobelx = static_cast<double>(subImg.at<uchar>(i + 1, j + 2) - subImg.at<uchar>(i + 1, j));
						double sobely = static_cast<double>(subImg.at<uchar>(i + 2, j + 1) - subImg.at<uchar>(i, j + 1));
						double gxx = sobelx * sobelx * m;
						double gxy = sobelx * sobely * m;
						double gyy = sobely * sobely * m;
						a += gxx;
						b += gxy;
						c += gyy;
						// p_i position.
						double px = static_cast<double>(j - HalfW);
						double py = static_cast<double>(i - HalfH);

						bb1 += gxx * px + gxy * py;
						bb2 += gxy * px + gyy * py;
					}
				}
				double det = a * c - b * b;
				if (fabs(det) <= DBL_EPSILON * DBL_EPSILON)
					break;
				// inverse
				double invA = c / det;
				double invC = a / det;
				double invB = -b / det;
				// new point
				cv::Point2f newPoint;
				newPoint.x = (float)(initx + HalfW + 1 + invA * bb1 + invB * bb2);
				newPoint.y = (float)(inity + HalfH + 1 + invB * bb1 + invC * bb2);
				// update err.
				err = (newPoint.x - iterPoint.x) * (newPoint.x - iterPoint.x) + (newPoint.y - iterPoint.y) * (newPoint.y - iterPoint.y);
				// update new point.
				iterPoint = newPoint;
				iterCnt++;
				if (iterPoint.x < 0 || iterPoint.x >= imgWidth || iterPoint.y < 0 || iterPoint.y >= imgHeight)
					break;
			}

			// verify convergence
			if (fabs(iterPoint.x - currPoint.x) > HalfW || fabs(iterPoint.y - currPoint.y) > HalfH) {
				iterPoint = currPoint;
			}
			
			// save the results.
			mC.emplace_back(iterPoint);
		}

		ArucoContainer mCTemp(intAC.id, mC);
		floatACs.emplace_back(mCTemp);
	}

	return floatACs;
}

float Detector::fromDistance2FV() const {
	float bestFV;
	bestFV = 49.8;
	return bestFV;
}

void Detector::showResult(const std::vector<cv::Point3f>& pointsM, const std::vector<cv::Point2f>& pointsP) const {

	float cost = 0.0;

	Corners reproPoints;
	reproPoints = getReprojectImagePoint(pointsM);
	cv::Mat imgResized;
	cv::cvtColor(capturedImg, imgResized, cv::COLOR_GRAY2BGR); // CV_8UC3
	
	std::vector<cv::Point3f> coordi_crs3d;
	coordi_crs3d.emplace_back(0.0, 0.0, 0.0);
	coordi_crs3d.emplace_back(60.0, 0.0, 0.0);
	coordi_crs3d.emplace_back(0.0, 60.0, 0.0);
	coordi_crs3d.emplace_back(0.0, 0.0, 60.0);
	Corners coordi_crs;
	coordi_crs = getReprojectImagePoint(coordi_crs3d);

	/*predefined parameters*/
	double resize_scale = 0.3;
	double text_scale = 1.5;
	int point_radius = 8;
	int line_width = 4;
	// r, g, b
	cv::Scalar color_arr[5] = { cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0), cv::Scalar(192, 112, 0), cv::Scalar(0, 255, 255) };
	
	// draw corners
	for (int i = 0; i < pointsP.size(); i++) {
		cv::circle(imgResized, Corner(reproPoints.at(i).x, reproPoints.at(i).y), point_radius, color_arr[3], -1);
		cost += pow(reproPoints.at(i).x - pointsP.at(i).x, 2) + pow(reproPoints.at(i).y - pointsP.at(i).y, 2);
	}
	cost = sqrt(cost / pointsP.size());

	// draw coordinate system
	if (coordi_crs.size() == 4)
	{
		if ((coordi_crs.at(0).x > 0 && coordi_crs.at(0).x < imgResized.cols - 1) &&
			(coordi_crs.at(0).y > 0 && coordi_crs.at(0).y < imgResized.rows - 1) &&
			(coordi_crs.at(1).x > 0 && coordi_crs.at(1).x < imgResized.cols - 1) &&
			(coordi_crs.at(1).y > 0 && coordi_crs.at(1).y < imgResized.rows - 1) &&
			(coordi_crs.at(2).x > 0 && coordi_crs.at(2).x < imgResized.cols - 1) &&
			(coordi_crs.at(2).y > 0 && coordi_crs.at(2).y < imgResized.rows - 1) &&
			(coordi_crs.at(3).x > 0 && coordi_crs.at(3).x < imgResized.cols - 1) &&
			(coordi_crs.at(3).y > 0 && coordi_crs.at(3).y < imgResized.rows - 1))
		{
			cv::line(imgResized, Corner(coordi_crs.at(0).x, coordi_crs.at(0).y), Corner(coordi_crs.at(1).x, coordi_crs.at(1).y), color_arr[0], line_width);
			cv::line(imgResized, Corner(coordi_crs.at(0).x, coordi_crs.at(0).y), Corner(coordi_crs.at(2).x, coordi_crs.at(2).y), color_arr[1], line_width);
			cv::line(imgResized, Corner(coordi_crs.at(0).x, coordi_crs.at(0).y), Corner(coordi_crs.at(3).x, coordi_crs.at(3).y), color_arr[2], line_width);
		}
	}

	std::string str1, str2;
	str1 = "position [" + std::to_string(markerPose.tvec[0]) + "," + std::to_string(markerPose.tvec[1]) + "," + std::to_string(markerPose.tvec[2]) +
		"], orientation [" + std::to_string(markerPose.rvec[0]) + "," + std::to_string(markerPose.rvec[1]) + "," + std::to_string(markerPose.rvec[2]) + "]";
	str2 = "cost: " + std::to_string(cost);

	cv::putText(imgResized, str1, cv::Point_<int>(10, 50), cv::FONT_HERSHEY_COMPLEX, text_scale, color_arr[4], 2);
	cv::putText(imgResized, str2, cv::Point_<int>(10, 110), cv::FONT_HERSHEY_COMPLEX, text_scale, color_arr[4], 2);
	cv::resize(imgResized, imgResized, cv::Size(), resize_scale, resize_scale);

	cv::imshow("repro", imgResized);
	cv::waitKey(1);
	//saveImage(imgResized, "repro");
	return;
}

void Detector::saveImage(const cv::Mat& image, const std::string& strPrefix) const {
	std::string fullName;
	SYSTEMTIME sysTime;
	GetLocalTime(&sysTime);

	std::string str_minute, str_second, str_millisecond;
	str_minute = std::to_string(sysTime.wMinute);
	if (str_minute.length() == 1) {
		str_minute = "0" + str_minute;
	}
	str_second = std::to_string(sysTime.wSecond);
	if (str_second.length() == 1) {
		str_second = "0" + str_second;
	}
	str_millisecond = std::to_string(sysTime.wMilliseconds);
	if (str_millisecond.length() == 1) {
		str_millisecond = "00" + str_millisecond;
	}
	else if (str_millisecond.length() == 2) {
		str_millisecond = "0" + str_millisecond;
	}
		
	fullName = strFilePath + strPrefix + str_minute + str_second + str_millisecond + ".bmp";
	cv::imwrite(fullName, image);
	return;
}

std::vector<cv::Point3f> Detector::registerPlane() {
	/*coares to fine has been achieved.*/
	std::vector<cv::Point3f> collidePoints;
	int amount = 50;  // every 200ms, save a result.

	// keep on emplace_back, until enough.
	while (true) {
		updateCapturedImg(GxHandler->getImgFlow());

		// detect marker
		cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
		std::vector<std::vector<cv::Point2f>> markersCornersRaw;
		cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
		std::vector<int> markersIdRaw;
		cv::aruco::detectMarkers(capturedImg, dictionary, markersCornersRaw, markersIdRaw, parameters);

		// if at least one marker detected
		if (markersIdRaw.size() == 0) {
			std::cout << "no marker..." << std::endl;
			continue;
		}
		std::vector<std::vector<cv::Point2f>> markersCorners;
		std::vector<int> markersId;
		auto first = markersIdRaw.begin();
		auto ff = markersCornersRaw.begin();
		while (first != markersIdRaw.end()) {
			bool unique = true;
			auto second = first + 1;
			auto ss = ff + 1;
			while (second != markersIdRaw.end()) {
				if (*second == *first) {
					unique = false;
					if ((fabs(ff->at(0).x - ff->at(1).x) + fabs(ff->at(0).y - ff->at(3).y)) > (fabs(ss->at(0).x - ss->at(1).x) + fabs(ss->at(0).y - ss->at(3).y))) {
						markersId.emplace_back(*second);
						markersCorners.emplace_back(*ss);
					}
					else {
						markersId.emplace_back(*first);
						markersCorners.emplace_back(*ff);
					}
					markersIdRaw.erase(second);
					markersCornersRaw.erase(ss);
					break;
				}
				second++;
				ss++;
			}
			if (unique) {
				markersId.emplace_back(*first);
				markersCorners.emplace_back(*ff);
			}
			first++;
			ff++;
		}

		ArucoContainers ACsTemp;
		int mCSi = 0;
		for (const std::vector<cv::Point2f>& mCS : markersCorners) {
			std::vector<cv::Point2f > mC;
			mC.emplace_back(mCS.at(0).x, mCS.at(0).y);
			mC.emplace_back(mCS.at(1).x, mCS.at(1).y);
			mC.emplace_back(mCS.at(2).x, mCS.at(2).y);
			mC.emplace_back(mCS.at(3).x, mCS.at(3).y);
			ArucoContainer mCTemp(markersId[mCSi], mC);
			ACsTemp.emplace_back(mCTemp);
			mCSi++;
		}

		ArucoContainers Acs = subPixelCorners(ACsTemp); // subpixel location

		//cv::aruco::drawDetectedMarkers(imageCopy, markersCorners, markersId); // draw the results
		std::vector<cv::Vec3d> rvecs, tvecs;
		cv::aruco::estimatePoseSingleMarkers(markersCorners, markerSize / 1000, cameraMatrix, distCoeffs, rvecs, tvecs); //求解旋转矩阵rvecs和平移矩阵tvecs
		//std::cout << "R :" << rvecs[0] << std::endl;
		//std::cout << "T :" << tvecs[0] << std::endl;

		// emplace_back
		if (tvecs.size() == 1) {
			collidePoints.emplace_back(tvecs[0]);
		}
		else {
			continue;
		}

		// break criteria
		if (collidePoints.size() > amount) {
			break;
		}

	}

	return collidePoints;
}

//pose3d Detector::detectTrolley() {
//	if (markerPose.tvec[2] == 0) {
//		// coarse to fine
//		bool coarseSuccess = false;
//		while (!coarseSuccess) {
//			coarseSuccess = coarseToFine(46.8, 50.9);
//		}
//	}
//	else {
//		float fv = fromDistance2FV();
//		caspHandler->setFocusVoltage(fv);
//	}
//
//	bool trackSuccess = false;
//	while (!trackSuccess) {
//		trackSuccess = monitor.estimatePose(true); // first estimate.
//	}
//
//
//
//}