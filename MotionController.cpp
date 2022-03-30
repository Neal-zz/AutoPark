#include "MotionController.h"

MotionController::MotionController(const CartPara& cartParaIn)
	: cartPara(cartParaIn)
	, cartIniPos(Eigen::Vector3d::Zero())
	, cartCurrPos(Eigen::Vector3d::Zero())
	, tarPos(Eigen::Vector3d::Zero())
	//, cartCurrVel(Eigen::Vector2d::Zero())
	//, wheelsVel(Eigen::Vector2d::Zero())
	, collidePlane(Eigen::Vector4d::Zero())  // bed boundary plane.
	, trolleyPlane(Eigen::Vector4d::Zero())
	, projectM(Eigen::Matrix<double, 3, 4>::Zero())
	//, caspHandler(caspian)
	//, GxHandler(GxCamera)
{
	trocarPose.rvec = cv::Vec3d(0.0,0.0,0.0);
	trocarPose.tvec = cv::Vec3d(0.0, 0.0, 0.0);
}

/* forward control test: input cartVel, get the motion trace. */
//void MotionController::forwardControl() {
//
//	/* set motion parameters */
//	updateCartIniPos(Eigen::Vector3d(1000.0,4000.0,EIGEN_PI/2));
//	roadSign.emplace_back(Eigen::Vector2d(1000.0, 4000.0));
//	updateCartCurrVel(Eigen::Vector2d(30, 0.1));
//
//	//std::ofstream outfile;
//	//outfile.open("fCData.txt", std::ios::trunc | std::ios::out);
//
//	int maxSimTime = 200;
//	for (int i = 0; i <= cartPara.communFreq * maxSimTime; i++)
//	{
//		Eigen::Vector2d wheelsVeli = fromCartVel2WheelsVel2();
//		updateWheelsVel(wheelsVeli);
//		Eigen::Vector3d cartAcci = fromWheelsVel2CartVel3();
//		Eigen::Vector3d cartPosNi;
//		Eigen::Vector3d cartPosi = getCartCurrPos();
//		cartPosNi << cartAcci(0) / cartPara.communFreq + cartPosi(0),
//			cartAcci(1) / cartPara.communFreq + cartPosi(1),
//			cartAcci(2) / cartPara.communFreq + cartPosi(2);
//		updateCartCurrPos(cartPosNi);
//		/*outfile << cartPosi(0) << '\t' << cartPosi(1) << '\t' << cartPosi(2) << '\n';*/
//		if ((cartPosNi.block<2, 1>(0, 0) - roadSign.at(roadSign.size() - 1)).norm() > 100) {
//			roadSign.emplace_back(Eigen::Vector2d(cartPosNi(0), cartPosNi(1)));
//		}
//		updateImg();
//	}
//	/*outfile.close();*/
//	return;
//}

/* closedLoopControl */
//void MotionController::closedLoopControl() {
//	/* set motion parameters */
//	updateCartIniPos(Eigen::Vector3d(2000.0, 5000.0, EIGEN_PI));
//	updateTarPos(Eigen::Vector3d(0, 1000.0, -80 * EIGEN_PI / 180));
//	std::vector<Eigen::Vector2d> pathRaw = pathGenerator();
//	fromDesired2FactualPath(pathRaw);
//	updateImg(pathRaw);
//
//	/* start motion.*/
//
//
//}

/* register the target trocar pose*/
bool MotionController::registerTrocar() {
	pose3d poseTemp;

	/*
	大博士的代码集成于此
	*/

	//updateTrocarPose(poseTemp);


	trocarPose.tvec = cv::Vec3d(1000, 500, 1200);
	trocarPose.rvec = cv::Vec3d(-4, -2, -1);
	return true;
}

 /* register the collide: bed boundary*/
bool MotionController::registerCollidePlane(const std::vector<cv::Point3f>& planePoints) {
	Eigen::Vector4d bedPlane = ransacPlane(planePoints, 10);  // threshold = 10mm.
	updateCollide(bedPlane);
	return true;
}


/* update trolly plane in each iteration*/
bool MotionController::registerTrolleyPlane(Detector& det) {
	//auto loadTrolleyTrajectory = [&]() {
	//	std::vector<cv::Point3f> tPs;
	//	{
	//		std::lock_guard<std::mutex> locker(det.tT_mutex);
	//		tPs = det.trolleyTrajectory;
	//	}

	//	return tPs;
	//};
	
	/*trolleyPoints = loadTrolleyTrajectory();*/
	trolleyPoints = det.trolleyTrajectory;
	if (trolleyPoints.size() > 2) {
		return false;
	}

	//Eigen::Vector4d trolleyPlane = ransacPlane(trolleyPoints, 5);  // threshold = 5mm
	Eigen::Vector4d trollyPlane = svdPlane(trolleyPoints);  // may be svd is more reasonable?
	updateTrolleyPlane(trolleyPlane);

	return true;
}

bool MotionController::registerTrolleyPlane(const std::vector<cv::Point3f>& planePoints) {
	Eigen::Vector4d Plane = ransacPlane(planePoints, 10);  // threshold = 10mm.
	updateTrolleyPlane(Plane);
	updateProjectM();
	return true;
}

Eigen::Vector4d MotionController::ransacPlane(const std::vector<cv::Point3f>& pts_3d, float threshold) const {
	auto ransacIter = [](const float& conf) {
		float err = 0.4;  // outlier percentage

		float num = log(1 - conf);
		float denum = log(1.0 - pow(1.0 - err, 3));  // 3: 3 points.
		return round(num / denum);
	};

	srand(time(0));  // rand seed
	float confidence = 0.9;
	int maxIter = ransacIter(confidence);
	float maxSamplePercent = 0.8f;  // maximum percentage control iteration break
	int maxSampleNum = ceil(static_cast<float>(pts_3d.size()) * maxSamplePercent);
	int n_inlier = 0;  // number of inliers.

	// start iteration
	Eigen::Vector4d plane;  // plane parameter.
	for (int i = 0; i < maxIter; i++) {
		// random number
		std::vector<int> index;
		for (int k = 0; k < 3; ++k)
		{
			index.push_back(rand() % pts_3d.size());
		}
		float x1 = pts_3d.at(index[0]).x, y1 = pts_3d.at(index[0]).y, z1 = pts_3d.at(index[0]).z;
		float x2 = pts_3d.at(index[1]).x, y2 = pts_3d.at(index[1]).y, z2 = pts_3d.at(index[1]).z;
		float x3 = pts_3d.at(index[2]).x, y3 = pts_3d.at(index[2]).y, z3 = pts_3d.at(index[2]).z;

		float a, b, c, d;  // plane parameter.
		a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
		b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
		c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
		d = -(a * x1 + b * y1 + c * z1);

		// if 3 points on one line, continue.
		if (a == 0 && b == 0 && c == 0) {
			continue;
		}

		// calculate inliers.
		index.clear();
		for (auto iter = pts_3d.begin(); iter != pts_3d.end(); ++iter)
		{
			float dis = fabs(a * iter->x + b * iter->y + c * iter->z + d) / sqrt(a * a + b * b + c * c);
			if (dis < threshold) {
				index.push_back(iter - pts_3d.begin());
			}
		}
		// update
		if (index.size() > n_inlier) {
			n_inlier = index.size();
			plane[0] = a;
			plane[1] = b;
			plane[2] = c;
			plane[3] = d;
		}
		if (n_inlier > maxSampleNum) {
			break;
		}
		//std::cout << "+1" << std::endl;
	}

	return plane;
}

Eigen::Vector4d MotionController::svdPlane(const std::vector<cv::Point3f>& plane_pts) const {
	Eigen::Vector3d center = Eigen::Vector3d::Zero();
	for (const auto& pt : plane_pts) {
		center[0] += static_cast<double>(pt.x);
		center[1] += static_cast<double>(pt.y);
		center[2] += static_cast<double>(pt.z);
	}
	center /= plane_pts.size();

	Eigen::MatrixXd A(plane_pts.size(), 3);
	for (int i = 0; i < plane_pts.size(); i++) {
		A(i, 0) = static_cast<double>(plane_pts[i].x) - center[0];
		A(i, 1) = static_cast<double>(plane_pts[i].y) - center[1];
		A(i, 2) = static_cast<double>(plane_pts[i].z) - center[2];
	}

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
	const float a = svd.matrixV()(0, 2);
	const float b = svd.matrixV()(1, 2);
	const float c = svd.matrixV()(2, 2);
	const float d = -(a * center[0] + b * center[1] + c * center[2]);
	std::cout << a << b << c << d << std::endl;
	return Eigen::Vector4d(a, b, c, d);
}

void MotionController::registerCartIniPos(const pose3d& det) {
	cartIniPos = projectM * Eigen::Vector4d(det.tvec[0],
		det.tvec[1], det.tvec[2], 1.0);
	
	cv::Mat rotm;
	cv::Rodrigues(det.rvec, rotm);

	Eigen::Vector3d z_ori = projectM * Eigen::Vector4d(rotm.at<double>(0, 2),
		rotm.at<double>(1, 2), rotm.at<double>(2, 2), 1.0);
	cartIniPos[2] = atan2(z_ori[1], z_ori[0]);
	cartCurrPos = cartIniPos;

	return;
}

/* if collide plane exists, then we may need to adjust tarPos*/
void MotionController::updateTarPos() {
	// give z value, get the point in the line.
	auto twoPlane2oneLine = [](const Eigen::Vector4d& planeA, const Eigen::Vector4d& planeB, double z) {
		double a = planeA[0], b = planeA[1], c = planeA[2], d = planeA[3];
		double e = planeB[0], f = planeB[1], g = planeB[2], h = planeB[3];
		Eigen::Vector4d point(-(-d * f + b * h - c * f * z + b * g * z) / (b * e - a * f),
			-(d * e - a * h + c * e * z - a * g * z) / (b * e - a * f), z, 1);
		return point;
	};

	auto sign = [](const double& a) {
		if (a > 0) {
			return 1;
		}
		else if (a == 0) {
			return 0;
		}
		else {
			return -1;
		}
	};

	auto cross = [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
		double result = -a[1] * b[0] + a[0] * b[1];
		return result;
	};

	// cart parameter
	double cartAlpha = atan(cartPara.w / 2.0 / cartPara.l1);

	// bed and cart max angle
	Eigen::Vector3d trocar_t = projectM * Eigen::Vector4d(trocarPose.tvec[0], trocarPose.tvec[1], trocarPose.tvec[2], 1);
	Eigen::Vector3d boundary1 = projectM * twoPlane2oneLine(collidePlane, trolleyPlane, 0);
	Eigen::Vector3d boundary2 = projectM * twoPlane2oneLine(collidePlane, trolleyPlane, 300);
	Eigen::Vector2d vec1 = boundary1.block<2, 1>(0, 0) - boundary2.block<2, 1>(0, 0);  // vector parallel with bed boundary.
	if (vec1[0] > 0) {  // points to the right
		vec1 = vec1 / vec1.norm();
	}
	else {
		vec1 = -vec1 / vec1.norm();
	}

	Eigen::Vector2d vec2 = boundary1.block<2, 1>(0, 0) - trocar_t.block<2, 1>(0, 0);
	double bedAlpha = acos(fabs(cross(vec1, vec2)) / sqrt(cartPara.l1 * cartPara.l1 + cartPara.w * cartPara.w / 4.0));

	// trocar and bed angle
	cv::Mat trocar_rotm;
	cv::Rodrigues(trocarPose.rvec, trocar_rotm);
	Eigen::Vector3d trocar_z = projectM * Eigen::Vector4d(trocar_rotm.at<double>(0, 2), trocar_rotm.at<double>(1, 2), trocar_rotm.at<double>(2, 2), 1);  // z orientation.
	Eigen::Vector2d vecTrocar(trocar_z[0], trocar_z[1]);
	double trocarAlpha = fabs(asin(vec1.dot(vecTrocar) / vecTrocar.norm()));

	// update judgement
	if (trocarAlpha <= bedAlpha) {
		tarPos = trocar_t;
		tarPos[2] = atan2(trocar_z[1], trocar_z[0]);
	}
	else {
		tarPos = trocar_t;
		tarPos[2] = (atan2(vec1[1], vec1[0]) - EIGEN_PI / 2.0) + sign(atan2(trocar_z[1], trocar_z[0]) - (atan2(vec1[1], vec1[0]) - EIGEN_PI / 2.0)) * (bedAlpha - cartAlpha);
	}

	return;
}

void MotionController::updateCartCurrPos(const pose3d& cartCurrPosIn) {
	cartCurrPos = projectM * Eigen::Vector4d(cartCurrPosIn.tvec[0],
		cartCurrPosIn.tvec[1], cartCurrPosIn.tvec[2], 1.0);

	cv::Mat rotm;
	cv::Rodrigues(cartCurrPosIn.rvec, rotm);

	Eigen::Vector3d z_ori = projectM * Eigen::Vector4d(rotm.at<double>(0, 2),
		rotm.at<double>(1, 2), rotm.at<double>(2, 2), 1.0);
	cartCurrPos[2] = atan2(z_ori[1], z_ori[0]);
	return;
}


void MotionController::updateProjectM() {

	// img_x's and img_y's 3d coordinates.
	Eigen::Vector3d x_coordinate = Eigen::Vector3d(0, 0, 1).cross(trolleyPlane.block<3, 1>(0, 0));
	if (x_coordinate.dot(Eigen::Vector3d(1, 0, 0)) > 0) {  // has the same direction with x_camera.
		x_coordinate = x_coordinate / x_coordinate.norm();
	}
	else {
		x_coordinate = -x_coordinate / x_coordinate.norm();
	}
	//std::cout << x_coordinate << std::endl;
	Eigen::Vector3d y_coordinate = x_coordinate.cross(trolleyPlane.block<3, 1>(0, 0));
	if (y_coordinate.dot(Eigen::Vector3d(0, 0, 1)) > 0) {  // has the same direction with z_camera.
		y_coordinate = y_coordinate / y_coordinate.norm();
	}
	else {
		y_coordinate = -y_coordinate / y_coordinate.norm();
	}
	//std::cout << y_coordinate << std::endl;

	// get projection matrix.
	projectM = Eigen::Matrix<double, 3, 4>::Zero();
	projectM(2, 3) = 1;
	projectM.block<2, 1>(0, 0) = Eigen::Vector2d(Eigen::Vector3d(1, 0, 0).dot(x_coordinate),
		Eigen::Vector3d(1, 0, 0).dot(y_coordinate));
	projectM.block<2, 1>(0, 1) = Eigen::Vector2d(Eigen::Vector3d(0, 1, 0).dot(x_coordinate),
		Eigen::Vector3d(0, 1, 0).dot(y_coordinate));
	projectM.block<2, 1>(0, 2) = Eigen::Vector2d(Eigen::Vector3d(0, 0, 1).dot(x_coordinate),
		Eigen::Vector3d(0, 0, 1).dot(y_coordinate));

	return;
}


/* generate roadSign?*/
bool MotionController::pathGenerator() {

	Eigen::Vector3d initPose = cartIniPos;
	initPose.block<2,1>(0,0) -= tarPos.block<2,1>(0,0);

	// y = x^2
	Eigen::Matrix<double, 2, 3> parabolic(Eigen::Matrix<double, 2, 3>::Zero()); // order: p, h, k
	parabolic(0, 0) = initPose(1) * initPose(1) / 4 / (initPose(0) - initPose(1) / tan(tarPos(2)));
	parabolic(0, 1) = -parabolic(0, 0) / tan(tarPos(2)) / tan(tarPos(2));
	parabolic(0, 2) = -2 * parabolic(0, 0) / tan(tarPos(2));
	parabolic(1, 0) = initPose(0) * initPose(0) / 4 / (initPose(1) - initPose(0) * tan(tarPos(2)));
	parabolic(1, 1) = -2 * parabolic(1, 0) * tan(tarPos(2));
	parabolic(1, 2) = -parabolic(1, 0) * tan(tarPos(2)) * tan(tarPos(2));

	// y = x^3
	const int angleNum = 13;
	Eigen::VectorXd initOri(angleNum);
	initOri << initPose(2) - 60 * EIGEN_PI / 180, initPose(2) - 50 * EIGEN_PI / 180, initPose(2) - 40 * EIGEN_PI / 180,
		initPose(2) - 30 * EIGEN_PI / 180, initPose(2) - 20 * EIGEN_PI / 180, initPose(2) - 10 * EIGEN_PI / 180,
		initPose(2) - 0 * EIGEN_PI / 180, initPose(2) + 10 * EIGEN_PI / 180, initPose(2) + 20 * EIGEN_PI / 180,
		initPose(2) + 30 * EIGEN_PI / 180, initPose(2) + 40 * EIGEN_PI / 180, initPose(2) + 50 * EIGEN_PI / 180,
		initPose(2) + 60 * EIGEN_PI / 180;
	Eigen::MatrixXd cubic1(angleNum, 4);
	Eigen::MatrixXd cubic2(angleNum, 4);
	Eigen::VectorXi cubic1State(angleNum); // 0: good; 1: sigular; 2:wrong direction.
	Eigen::VectorXi cubic2State(angleNum);
	cubic1State = Eigen::Matrix<int, angleNum, 1>::Zero();
	cubic2State = Eigen::Matrix<int, angleNum, 1>::Zero();
	for (int i = 0; i < angleNum; i++)
	{
		if (fabs(initOri(i) - EIGEN_PI / 2) < 0.001 || fabs(initOri(i) + EIGEN_PI / 2) < 0.001 || fabs(initPose(0)) < 0.001) // other criterions are discussed later.
			cubic1State(i) = 1;
		cubic1(i, 0) = 0;
		cubic1(i, 1) = tan(tarPos(2));
		cubic1(i, 2) = (3 * initPose(1) - initPose(0) * (tan(initOri(i)) + 2 * tan(tarPos(2)))) / initPose(0) / initPose(0);
		cubic1(i, 3) = (initPose(0) * (tan(initOri(i)) + tan(tarPos(2))) - 2 * initPose(1)) / initPose(0) / initPose(0) / initPose(0);
		if (fabs(initOri(i)) < 0.001 || fabs(fabs(initOri(i)) - EIGEN_PI) < 0.001 || fabs(initPose(1)) < 0.001)
			cubic2State(i) = 1;
		cubic2(i, 0) = 0;
		cubic2(i, 1) = 1 / tan(tarPos(2));
		cubic2(i, 2) = (3 * initPose(0) - initPose(1) * (1 / tan(initOri(i)) + 2 / tan(tarPos(2)))) / initPose(1) / initPose(1);
		cubic2(i, 3) = (initPose(1) * (1 / tan(initOri(i)) + 1 / tan(tarPos(2))) - 2 * initPose(0)) / initPose(1) / initPose(1) / initPose(1);
	}

	// generate roadSignRaw.
	//std::vector<Eigen::Vector2d> roadSignRaw;
	//for (int i = 0; i < 20; i++)
	//{
	//	double z2 = initPose(1) / 2 * (1 + cos(EIGEN_PI * i / 19));
	//	roadSignRaw.emplace_back(Eigen::Vector2d((z2 - parabolic(0, 2)) * (z2 - parabolic(0, 2)) / 4 / parabolic(0, 0) + parabolic(0, 1) + tarPos[0], z2 + tarPos[1]));
	//}

	//for (int i = 0; i < 20; i++)
	//{
	//	double z1 = initPose(0) / 2 * (1 + cos(EIGEN_PI * i / 19));
	//	roadSignRaw.emplace_back(Eigen::Vector2d(z1 + tarPos[0], (z1 - parabolic(1, 1)) * (z1 - parabolic(1, 1)) / 4 / parabolic(1, 0) + parabolic(1, 2) + tarPos[1]));
	//}

	//for (int pathId = 0; pathId < angleNum; pathId++) {  // angleNum
	//	for (int i = 0; i < 20; i++)
	//	{
	//		double z1 = initPose(0) / 2 * (1 + cos(EIGEN_PI * i / 19));
	//		roadSignRaw.emplace_back(Eigen::Vector2d(z1 + tarPos[0], cubic1(pathId, 0) + cubic1(pathId, 1) * z1 + cubic1(pathId, 2) * z1 * z1 + cubic1(pathId, 3) * z1 * z1 * z1 + tarPos[1]));
	//	}
	//}

	for (int pathId = 10; pathId < 11; pathId++) {  // angleNum
		for (int i = 0; i < 20; i++)
		{
			double z2 = initPose(1) / 2 * (1 + cos(EIGEN_PI*i / 19));
			roadSign.emplace_back(Eigen::Vector2d(cubic2(pathId, 0) + cubic2(pathId, 1) * z2 + cubic2(pathId, 2) * z2 * z2 + cubic2(pathId, 3) * z2 * z2 * z2 + tarPos[0], z2 + tarPos[1]));
		}
	}

	return true;
}

/* from generated path, get factual path.*/
std::vector<Eigen::Vector2d> MotionController::fromDesired2FactualPath() {
	//std::ofstream outfile; // log the "from desired to factual path data". First 20 rows match the sign post, others are trace.
	//outfile.open("fD2FPData.txt", std::ios::trunc | std::ios::out);

	// make sure that the desiredPath matches the cartIniPos and the tarPos.
	if (fabs(cartIniPos(0) - roadSign.at(0)[0]) > 0.001 || fabs(cartIniPos(1) - roadSign.at(0)[1]) > 0.001 || fabs(tarPos(0) - roadSign.at(roadSign.size() - 1)[0]) > 0.001 || fabs(tarPos(1) - roadSign.at(roadSign.size() - 1)[1]) > 0.001)
	{
		std::cerr << "iniPosIn or tarPosIn doesn't match the desiredPath!" << std::endl;
		return std::vector<Eigen::Vector2d>();
	}

	//for (int i = 0; i < 20; i++) // log the 20 signposts
	//{
	//	outfile << desiredPath(i, 0) << '\t' << desiredPath(i, 1) << '\n';
	//}

	/*parameter 0.05 needs to be adjusted.*/
	std::vector<Eigen::Vector2d> factualPath;
	double disThresh = 10; // 10mm
	//roadSign.emplace_back(Eigen::Vector2d(cartIniPos(0), cartIniPos(1)));
	int dpSign = 1; // desiredPath sign post (from 0-19).
	Eigen::Vector2d tarPosi(roadSign.at(dpSign)[0], roadSign.at(dpSign)[1]);
	Eigen::Vector3d cartPosi(cartIniPos);
	Eigen::Vector2d cartVeli(Eigen::Vector2d::Zero());
	Eigen::Vector2d wheelsVeli(Eigen::Vector2d::Zero());
	Eigen::Vector3d cartVel3i(Eigen::Vector3d::Zero());
	while ((sqrt(pow(cartPosi(0) - tarPos(0), 2) + pow(cartPosi(1) - tarPos(1), 2)) > disThresh))  // doesn't reach the target position.
	{
		double disErr = sqrt(pow(cartPosi(0) - tarPosi(0), 2) + pow(cartPosi(1) - tarPosi(1), 2));
		if (dpSign < 19)
		{
			double preErr = sqrt(pow(cartPosi(0) - roadSign.at(dpSign - 1)[0], 2) + pow(cartPosi(1) - roadSign.at(dpSign - 1)[1], 2));
			/*parameter 0.5 needs to be adjusted.*/
			if (disErr < preErr) // update tarPosi when get close to the previous one.
			{
				//roadSign.emplace_back(Eigen::Vector2d(cartPosi(0), cartPosi(1)));
				dpSign++;
				tarPosi = roadSign.at(dpSign);  // update tarPosi.
				disErr = sqrt(pow(cartPosi(0) - tarPosi(0), 2) + pow(cartPosi(1) - tarPosi(1), 2));
				//std::cout << cartPosi(0) << cartPosi(1) << std::endl;
			}
		}
		double angErr = atan2(tarPosi(1) - cartPosi(1), tarPosi(0) - cartPosi(0)) - cartPosi(2);
		if (angErr > EIGEN_PI) {
			angErr -= 2 * EIGEN_PI;
		}
		else if (angErr < -EIGEN_PI) {
			angErr += 2 * EIGEN_PI;
		}

		// compute next position and orientation from information at time i.
		cartVel3i = fromWheelsVel2CartVel3(cartPosi, wheelsVeli);
		Eigen::Vector3d cartPosNi;
		cartPosNi << cartVel3i(0) / cartPara.communFreq + cartPosi(0),
			cartVel3i(1) / cartPara.communFreq + cartPosi(1),
			cartVel3i(2) / cartPara.communFreq + cartPosi(2);
		cartPosi = cartPosNi;

		// comute next velocity.
		/* kp=1*/
		Eigen::Vector2d cartVelNi((std::min)({ disErr, cartPara.linVelLim }),
			fabs(angErr) > cartPara.angVelLim ? cartPara.angVelLim * angErr / fabs(angErr) : angErr);
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

		factualPath.emplace_back(Eigen::Vector2d(cartPosi(0), cartPosi(1)));
		//outfile << cartPosi(0) << '\t' << cartPosi(1) << '\n';
	}
	//outfile.close();
	factualPath.emplace_back(Eigen::Vector2d(tarPos(0), tarPos(1)));
	return factualPath;
}

//Eigen::Vector2d MotionController::fromCartVel2WheelsVel2() const {
//	Eigen::Vector2d wheelsVelOut; // (vR, vL).
//	wheelsVelOut << cartCurrVel(0) + cartCurrVel(1)*cartPara.wheelsDistance / 2,
//		cartCurrVel(0) - cartCurrVel(1)*cartPara.wheelsDistance / 2;
//	return wheelsVelOut;
//}

Eigen::Vector2d MotionController::fromCartVel2WheelsVel2(const Eigen::Vector2d& cartCurrVelIn) const {
	Eigen::Vector2d wheelsVelOut; // (vR, vL).
	wheelsVelOut << cartCurrVelIn(0) + cartCurrVelIn(1)*cartPara.wheelsDistance / 2,
		cartCurrVelIn(0) - cartCurrVelIn(1)*cartPara.wheelsDistance / 2;
	return wheelsVelOut;
}

//Eigen::Vector3d MotionController::fromWheelsVel2CartVel3() const {
//	Eigen::Vector3d cartVel3; // (xDot, yDot, thetaDot).
//	cartVel3 << cos(cartCurrPos(2))*(wheelsVel(0) + wheelsVel(1)) / 2,
//		sin(cartCurrPos(2))*(wheelsVel(0) + wheelsVel(1)) / 2,
//		(wheelsVel(0) - wheelsVel(1)) / cartPara.wheelsDistance;
//	return cartVel3;
//}

Eigen::Vector3d MotionController::fromWheelsVel2CartVel3(
	const Eigen::Vector3d& cartCurrPosIn, const Eigen::Vector2d& wheelsVelIn) const {
	Eigen::Vector3d cartVel3; // (xDot, yDot, thetaDot).
	cartVel3 << cos(cartCurrPosIn(2))*(wheelsVelIn(0) + wheelsVelIn(1)) / 2,
		sin(cartCurrPosIn(2))*(wheelsVelIn(0) + wheelsVelIn(1)) / 2,
		(wheelsVelIn(0) - wheelsVelIn(1)) / cartPara.wheelsDistance;
	return cartVel3;
}

void MotionController::updateImg(const std::vector<Eigen::Vector2d>& pathRaw) {
	// give z value, get the point in the line.
	auto twoPlane2oneLine = [](const Eigen::Vector4d& planeA, const Eigen::Vector4d& planeB, double z) {
		double a = planeA[0], b = planeA[1], c = planeA[2], d = planeA[3];
		double e = planeB[0], f = planeB[1], g = planeB[2], h = planeB[3];
		Eigen::Vector4d point(-(-d * f + b * h - c * f * z + b * g * z) / (b * e - a * f),
			-(d * e - a * h + c * e * z - a * g * z) / (b * e - a * f), z, 1);
		return point;
	};

	/* convert parameters from mm to pixel*/
	Eigen::Vector4d tP_temp = trolleyPlane;
	tP_temp(3) = tP_temp(3) / 10.0;
	if (trolleyPlane.block<3, 1>(0, 0).norm() == 0) {
		tP_temp = Eigen::Vector4d(0,1,0,0);
	}

	double trolley_r = cartPara.r / 10.0;
	double trolley_l1 = cartPara.l1 / 10.0;
	double trolley_w = cartPara.w / 10.0;
	double trolley_l2 = cartPara.l2 / 10.0;

	int halfWidth = 300;  // img size.
	int height = 700;
	cv::Mat img = cv::Mat(cv::Size(halfWidth * 2 + 1, height + 1), CV_8UC3, cv::Scalar(0, 0, 0));

	//std::cout << projectM << std::endl;

	// draw the boundary area.
	if (collidePlane[3] != 0) {
		Eigen::Vector4d cP_temp = collidePlane;
		cP_temp(3) = cP_temp(3) / 10.0;
		Eigen::Vector3d boundary1 = projectM * twoPlane2oneLine(cP_temp, tP_temp, 0);
		Eigen::Vector3d boundary2 = projectM * twoPlane2oneLine(cP_temp, tP_temp, 300);
		//std::cout << boundary1 << std::endl;
		//std::cout << boundary2 << std::endl;
		std::vector<cv::Point> contour1;
		//contour1.emplace_back(cv::Point(600, 699));
		contour1.emplace_back(cv::Point(halfWidth * 2, height - ((halfWidth - boundary1[0]) * (boundary2[1] - boundary1[1]) / (boundary2[0] - boundary1[0]) + boundary1[1])));
		contour1.emplace_back(cv::Point(0, height - ((-halfWidth - boundary1[0]) * (boundary2[1] - boundary1[1]) / (boundary2[0] - boundary1[0]) + boundary1[1])));
		if (contour1.at(0).y < height && contour1.at(1).y < height) {
			contour1.emplace_back(cv::Point(0, height));
			contour1.emplace_back(cv::Point(halfWidth * 2, height));
		}
		else if (contour1.at(0).y > contour1.at(1).y) {  // right > left
			contour1.emplace_back(cv::Point(contour1.at(1).x, contour1.at(0).y));
		}
		else {  // left > right
			contour1.emplace_back(cv::Point(contour1.at(0).x, contour1.at(1).y));
		}
		cv::Scalar color1(167, 112, 49);
		cv::polylines(img, contour1, true, color1, 2, cv::LINE_AA);
		cv::fillPoly(img, contour1, color1);
	}

	// draw trocar
	if (trocarPose.tvec[2] != 0) {
		cv::Mat trocar_rotm;
		cv::Rodrigues(trocarPose.rvec, trocar_rotm);
		Eigen::Vector4d trocar_z(trocar_rotm.at<double>(0, 2), trocar_rotm.at<double>(1, 2), trocar_rotm.at<double>(2, 2), 1);  // z orientation.
		Eigen::Vector4d trocar_t(trocarPose.tvec[0] / 10.0, trocarPose.tvec[1] / 10.0, trocarPose.tvec[2] / 10.0, 1);

		Eigen::Vector3d Trocar = projectM * trocar_t;
		cv::Scalar color2(17, 204, 254);
		cv::circle(img, cv::Point(halfWidth + Trocar[0], height - Trocar[1]), 6, color2, 2);
		Eigen::Vector3d TrocarZ = projectM * trocar_z;
		TrocarZ.block<2, 1>(0, 0) = TrocarZ.block<2, 1>(0, 0) / TrocarZ.block<2, 1>(0, 0).norm();
		cv::line(img, cv::Point(halfWidth + Trocar[0], height - Trocar[1]), cv::Point(halfWidth + Trocar[0] + 50 * TrocarZ[0], height - (Trocar[1] + 50 * TrocarZ[1])), color2, 2);
		double lineWidth = 20;
		double dashWidth = 60 * fabs(TrocarZ[0]) / TrocarZ.block<2, 1>(0, 0).norm();
		if (TrocarZ[0] > 0) {
			// draw dash from 0 to Trocar
			for (double i = Trocar[0] - dashWidth; i > -halfWidth; i -= dashWidth) {
				cv::Point point1(i, (i - Trocar[0]) * TrocarZ[1] / TrocarZ[0] + Trocar[1]);
				cv::line(img, cv::Point(halfWidth + point1.x, height - point1.y), cv::Point(halfWidth + point1.x + lineWidth * TrocarZ[0], height - (point1.y + lineWidth * TrocarZ[1])), color2, 2);
			}
		}
		else {
			// draw dash from Trocar to img width.
			for (double i = Trocar[0] + dashWidth; i < halfWidth; i += dashWidth) {
				cv::Point point1(i, (i - Trocar[0]) * TrocarZ[1] / TrocarZ[0] + Trocar[1]);
				//cv::Point point2(i - lineWidth, (i - lineWidth - Trocar[0]) * TrocarZ[1] / TrocarZ[0] + Trocar[1]);
				cv::line(img, cv::Point(halfWidth + point1.x, height - point1.y), cv::Point(halfWidth + point1.x + lineWidth * TrocarZ[0], height - (point1.y + lineWidth * TrocarZ[1])), color2, 2);
			}
		}
	}

	// draw trolley
	if (cartCurrPos[1] != 0) {
		Eigen::Vector3d Trolley = Eigen::Vector3d(cartCurrPos[0] /10.0, cartCurrPos[1]/10.0, 1.0);
		cv::Scalar color3(172, 181, 187);
		Eigen::Vector3d TrolleyZ = Eigen::Vector3d(cos(cartCurrPos[2]), sin(cartCurrPos[2]), 1.0);
		std::vector<cv::Point> contour3;
		contour3.emplace_back(cv::Point(halfWidth + Trolley[0] - TrolleyZ[1] * trolley_w / 2.0 - TrolleyZ[0] * trolley_l1, height - (Trolley[1] + TrolleyZ[0] * trolley_w / 2.0 - TrolleyZ[1] * trolley_l1)));
		contour3.emplace_back(cv::Point(halfWidth + Trolley[0] + TrolleyZ[1] * trolley_w / 2.0 - TrolleyZ[0] * trolley_l1, height - (Trolley[1] - TrolleyZ[0] * trolley_w / 2.0 - TrolleyZ[1] * trolley_l1)));
		contour3.emplace_back(cv::Point(halfWidth + Trolley[0] + TrolleyZ[1] * trolley_w / 2.0 - TrolleyZ[0] * (trolley_l1 + trolley_l2), height - (Trolley[1] - TrolleyZ[0] * trolley_w/2.0 - TrolleyZ[1] * (trolley_l1 + trolley_l2))));
		contour3.emplace_back(cv::Point(halfWidth + Trolley[0] - TrolleyZ[1] * trolley_w / 2.0 - TrolleyZ[0] * (trolley_l1 + trolley_l2), height - (Trolley[1] + TrolleyZ[0] * trolley_w/2.0 - TrolleyZ[1] * (trolley_l1 + trolley_l2))));
		cv::polylines(img, contour3, true, color3, 1, cv::LINE_AA);
		cv::fillPoly(img, contour3, color3);
		cv::Mat imgNew = img.clone();
		cv::circle(imgNew, cv::Point(halfWidth + Trolley[0], height - Trolley[1]), trolley_r, color3, -1);
		std::vector<cv::Point> contour2;
		contour2.emplace_back(cv::Point(halfWidth + Trolley[0] - TrolleyZ[1] * trolley_r, height - (Trolley[1] + TrolleyZ[0] * trolley_r)));
		contour2.emplace_back(cv::Point(halfWidth + Trolley[0] + TrolleyZ[1] * trolley_r, height - (Trolley[1] - TrolleyZ[0] * trolley_r)));
		contour2.emplace_back(cv::Point(halfWidth + Trolley[0] + TrolleyZ[1] * trolley_r - TrolleyZ[0] * trolley_l1, height - (Trolley[1] - TrolleyZ[0] * trolley_r - TrolleyZ[1] * trolley_l1)));
		contour2.emplace_back(cv::Point(halfWidth + Trolley[0] - TrolleyZ[1] * trolley_r - TrolleyZ[0] * trolley_l1, height - (Trolley[1] + TrolleyZ[0] * trolley_r - TrolleyZ[1] * trolley_l1)));
		cv::polylines(imgNew, contour2, true, color3, 1, cv::LINE_AA);
		cv::fillPoly(imgNew, contour2, color3);
		cv::addWeighted(img, 0.5, imgNew, 0.5, 0.0, img);
	}

	// draw roadSign, 20 points
	if (roadSign.size() != 0) {
		cv::Scalar color4(78, 194, 150);
		for (int i = 0; i < roadSign.size(); i++) {
			cv::circle(img, cv::Point(halfWidth + roadSign.at(i)[0]/10.0, height - roadSign.at(i)[1]/10.0), 2, color4, -1);
		}
	}

	// draw trajectory
	if (pathRaw.size() != 0) {
		cv::Scalar color2(17, 204, 254);
		for (int i = 0; i < pathRaw.size(); i++) {
			cv::circle(img, cv::Point(halfWidth + pathRaw.at(i)[0] / 10.0, height - pathRaw.at(i)[1] / 10.0), 2, color2, -1);
		}
	}

	// draw tarpose
	if (tarPos[1] != 0) {
		cv::Scalar color4(78, 194, 150);
		cv::line(img, cv::Point(halfWidth + tarPos[0] / 10.0, height - tarPos[1] / 10.0), cv::Point(halfWidth + tarPos[0] / 10.0 + 50 * cos(tarPos[2]), height - (tarPos[1] / 10.0 + 50 * sin(tarPos[2]))), color4, 2);
	}
	
	cv::imwrite("1.png", img);
	cv::imshow("sss", img);
	cv::waitKey(0);

}
