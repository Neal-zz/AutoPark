#include "Detector.h"


Detector::Detector(const cv::Size& size)
	: SIZE(size)
	, SIGMA(2)
	, HALF_PATCH_SIZE(3)
	, PATCH_X(calcPatchX())
	, WIDTH_MIN(20)
	, CORR_THRESHOLD(0.7f) // 0.7f
	, rect({ cv::Range(0, size.width), cv::Range(0, size.height) })
{
	initWidth = WIDTH_MIN;

}

QRsTemplate Detector::process(const cv::Mat& image)
{
	cv::Mat image_roi = image(rect.range_y, rect.range_x).clone();  // TODO: image_roi may be used later.
	QRsTemplate QRs;
	bool is_vaild = detectCorners(image_roi, QRs); // corners: all finded corners.

	// Then we update the ROI.
	//if (is_vaild)
	//{
	//	for (CornerTemplate& corner : corners)
	//		corner.point += Corner(rect.range_x.start, rect.range_y.start);

	//	PixelType width_sum = 0;
	//	Corner point_sum(0, 0);
	//	for (const CornerTemplate& corner : corners)
	//	{
	//		point_sum += corner.point;
	//		width_sum += corner.width;
	//	}
	//	PixelType px_avg = point_sum.x / corners.size();
	//	PixelType py_avg = point_sum.y / corners.size();
	//	PixelType width_avg = width_sum / corners.size();

	//	PixelType rect_side = round(width_avg * 20);
	//	rect.range_x = cv::Range(
	//		std::max(static_cast<int>(px_avg - rect_side / 2), 0),
	//		std::min(static_cast<int>(px_avg + rect_side / 2) + 1, image.cols));
	//	rect.range_y = cv::Range(
	//		std::max(static_cast<int>(py_avg - rect_side / 2), 0),
	//		std::min(static_cast<int>(py_avg + rect_side / 2) + 1, image.rows));
	//}
	//else
	//{
	//	rect.range_x = cv::Range(0, SIZE.width);
	//	rect.range_y = cv::Range(0, SIZE.height);
	//}

	//Corners res;
	//for (const QRTemplate& p : QRs) {
	//	res.emplace_back(p.corner0);
	//	res.emplace_back(p.corner1);
	//	res.emplace_back(p.corner2);
	//	res.emplace_back(p.corner3);
	//}

	return QRs;
}

bool Detector::detectCorners(const cv::Mat& gray_image_in, QRsTemplate& QRs_selected)
{
	gray_image = gray_image_in;

	secondDerivCornerMetric(I_angle, I_weight, cmax);

	Maximas corners = nonMaximumSuppression(cmax, ceil(initWidth / 2), ceil(initWidth / 2));
	std::sort(corners.begin(), corners.end(),
		[](const Maxima& lhs, const Maxima& rhs) { return lhs.val > rhs.val; });

	/*monitor all the corners detected.*/
	/*monitor all the chessboard corners.*/
	//corners_on_marker.clear();
	//for (Maxima& m : corners) {
	//	CornerTemplate corner_first;
	//	corner_first.point = subPixelLocation(m.corner);
	//	corner_first.width = initWidth;

	//	PixelType angle1, angle2;
	//	findEdgeAngles(corner_first.point, angle1, angle2);
	//	if (abs(angle1) < 1e-7 && abs(angle2) < 1e-7)
	//		continue;

	//	PixelType template_angle = (angle1 + angle2 - CV_PI / 2) / 2;
	//	PixelType corr = calcBolicCorrelation(corner_first.point, WIDTH_MIN / 2, template_angle);
	//	if (corr <= CORR_THRESHOLD)
	//		continue;

	//	corners_on_marker.emplace_back(m.corner, WIDTH_MIN);
	//	std::cout << " " << m.corner.x << " " << m.corner.y << std::endl;
	//	//std::cout << angle1 << " " << angle2 << std::endl;
	//}
	//bool is_valid = true;
	
	//auto t3 = tic();
	bool is_valid = detectCornersOnMarker(corners, QRs_selected);
	//toc(t3);

	return is_valid;
}

void Detector::secondDerivCornerMetric(cv::Mat& I_angle, cv::Mat& I_weight, cv::Mat& cmax)
{
	cv::Mat gaussian_image;
	cv::GaussianBlur(gray_image, gaussian_image, cv::Size(7 * SIGMA + 1, 7 * SIGMA + 1), SIGMA);

	cv::Mat dx = (cv::Mat_<PixelType>(1, 3) << -1, 0, 1);
	cv::Mat dy;
	cv::transpose(dx, dy);

	// first derivative
	cv::Mat Ix = conv2(gaussian_image, dx, "same");
	cv::Mat Iy = conv2(gaussian_image, dy, "same");
	cv::Mat I_45 = Ix * cos(CV_PI / 4) + Iy * sin(CV_PI / 4);
	cv::Mat I_n45 = Ix * cos(-CV_PI / 4) + Iy * sin(-CV_PI / 4);

	// second derivative
	cv::Mat Ixy = conv2(Ix, dy, "same");
	cv::Mat I_45_x = conv2(I_45, dx, "same");
	cv::Mat I_45_y = conv2(I_45, dy, "same");
	cv::Mat I_45_45 = I_45_x * cos(-EIGEN_PI / 4) + I_45_y * sin(-EIGEN_PI / 4);

	cv::Mat cxy = static_cast<cv::Mat>(pow(SIGMA, 2) * cv::abs(Ixy) - 1.5 * SIGMA * (cv::abs(I_45) + cv::abs(I_n45)));
	cv::Mat c45 = static_cast<cv::Mat>(pow(SIGMA, 2) * cv::abs(I_45_45) - 1.5 * SIGMA * (cv::abs(Ix) + cv::abs(Iy)));
	cmax = static_cast<cv::Mat>(cv::max(cxy, c45));
	cv::Mat zeros_mat = cv::Mat::zeros(cmax.size(), MatType);
	cmax = cv::max(cmax, zeros_mat);

	cv::phase(Ix, Iy, I_angle);
	cv::magnitude(Ix, Iy, I_weight);

	return;
}

Maximas Detector::nonMaximumSuppression(const cv::Mat& img, int n, int margin, PixelType tau)
{
	// img is cmax. n is initWidth/2. margin is initWidth/2.
	int width = img.cols;
	int height = img.rows;

	Maximas maxima;
	for (int i = margin; i < width - margin - n; i += n + 1)
	{
		for (int j = margin; j < height - margin - n; j += n + 1)
		{
			int max_i = i;
			int max_j = j;
			PixelType max_val = img.ptr<PixelType>(j)[i];

			for (int i2 = i; i2 <= i + n; ++i2)
			{
				for (int j2 = j; j2 <= j + n; ++j2)
				{
					PixelType curr_val = img.ptr<PixelType>(j2)[i2];
					if (curr_val > max_val)
					{
						max_i = i2;
						max_j = j2;
						max_val = curr_val;
					}
				}
			}

			if (max_val < tau) // is not corner.
				continue;

			bool failed = false;
			for (int i2 = max_i - n; i2 <= max_i + n; i2++)
			{
				for (int j2 = max_j - n; j2 <= max_j + n; j2++)
				{
					if (img.ptr<PixelType>(j2)[i2] > max_val)
					{
						failed = true;
						break;
					}
				}
				if (failed)
					break;
			}

			if (failed)
				continue;

			maxima.emplace_back(max_i, max_j, max_val);
		}
	}

	return maxima;
}

bool Detector::detectCornersOnMarker(const Maximas& corners, QRsTemplate& QRs_selected)
{
	// corners is nonMaximumSuppression corners.
	bool QRFound=false;
	QRs_selected.clear();  // initial.
	CornersTemplate cornersLeft_selected, cornersRight_selected;
	for (const Maxima& p : corners)
	{

		bool continueFlag = false;
		for (const QRTemplate& q : QRs_selected) {
			if (abs(p.corner.x - q.corner0.x) + abs(p.corner.y - q.corner0.y) < 2 * cv::norm(q.corner0 - q.corner1) ||
				abs(p.corner.x - q.corner1.x) + abs(p.corner.y - q.corner1.y) < 2 * cv::norm(q.corner0 - q.corner1) ||
				abs(p.corner.x - q.corner2.x) + abs(p.corner.y - q.corner2.y) < 2 * cv::norm(q.corner0 - q.corner1) ||
				abs(p.corner.x - q.corner3.x) + abs(p.corner.y - q.corner3.y) < 2 * cv::norm(q.corner0 - q.corner1)) {
				continueFlag = true;
				break;
			}
		}
		if (continueFlag) continue;

		cornersLeft_selected.clear(); // initial
		cornersRight_selected.clear();

		CornerTemplate corner_first, corner_second;
		bool secondFound = findFirstSecondCorners(p.corner, corner_first, corner_second); // input p.corner. get corner_first, corner_seconde and dir.

		if (!secondFound) continue;

		if (corner_first.point.x > corner_second.point.x)
			std::swap(corner_first, corner_second);

		cornersLeft_selected.emplace_back(corner_first);
		cornersRight_selected.emplace_back(corner_second);

		// leftline search.
		std::array<std::pair<int, CornerTemplate>, 2> comps = {
			std::make_pair(-1, corner_first), // upwards.
			std::make_pair(1, corner_first) }; // downwards.
		for (std::pair<int, CornerTemplate>& comp : comps)
		{
			while (true)
			{
				///*perpendicular search.*/
				//std::array<std::pair<int, CornerTemplate>, 2> compsPerp = {
				//	std::make_pair(1, comp.second),
				//	std::make_pair(-1, comp.second) }; // two perpendicular direction.
				//for (std::pair<int, CornerTemplate>& compPerp : compsPerp) {
				//	while (true) {
				//		CornerTemplate cornerPerp_next = predictPerpNextCorner(compPerp.second, compPerp.first);
				//		if (cornerPerp_next.corr <= CORR_THRESHOLD)
				//			break;
				//		compPerp.second = cornerPerp_next;
				//		corners_selected.emplace_back(cornerPerp_next);
				//	}
				//}

				CornerTemplate corner_next = predictNextCorner(comp.second, comp.first);
				if (corner_next.corr <= CORR_THRESHOLD)
					break;

				comp.second = corner_next;
				if (abs(comp.first - 1) < 0.1)
					cornersLeft_selected.emplace(cornersLeft_selected.begin(), corner_next);
				else
					cornersLeft_selected.emplace_back(corner_next);
			}
		}

		if (cornersLeft_selected.size() < 2) {
			continue;
		}

		comps.at(0).second = corner_second;
		comps.at(1).second = corner_second;
		for (std::pair<int, CornerTemplate>& comp : comps)
		{
			while (true)
			{
				CornerTemplate corner_next = predictNextCorner(comp.second, comp.first);
				if (corner_next.corr <= CORR_THRESHOLD) // 这儿多找到一个，但是理应不是棋盘格角点
					break;

				comp.second = corner_next;
				if (abs(comp.first - 1) < 0.1) // downwards.
					cornersRight_selected.emplace(cornersRight_selected.begin(), corner_next);
				else // upwards.
					cornersRight_selected.emplace_back(corner_next);
			}
		}

		if (cornersRight_selected.size() < 2) {
			continue;
		}


		/*only search once.*/
		if (cornersLeft_selected.size() < 4 && cornersRight_selected.size() < 4)
		{
			QRTemplate QR_next;
			QR_next.corner0 = cornersLeft_selected.at(0).point;
			QR_next.corner1 = cornersRight_selected.at(0).point;
			QR_next.corner2 = cornersLeft_selected.at(1).point;
			QR_next.corner3 = cornersRight_selected.at(1).point;

			//if (abs(QR_next.corner0.y - QR_next.corner1.y) > abs(QR_next.corner0.y - QR_next.corner2.y) / 2 ||
			//	abs(QR_next.corner2.y - QR_next.corner3.y) > abs(QR_next.corner0.y - QR_next.corner2.y) / 2) {
			//	continue;
			//}


			QR_next.id = getQRID(QR_next);
			//if (QR_next.id == -1) {
			//	continue;
			//}
		
			QRs_selected.emplace_back(QR_next);
			std::cout << cornersLeft_selected.at(0).width << std::endl;
			std::cout << cornersLeft_selected.at(1).width << std::endl;
			std::cout << cornersRight_selected.at(0).width << std::endl;
			std::cout << cornersRight_selected.at(1).width << std::endl;
			//std::cout << QRs_selected.size() << std::endl;
			std::cout << QR_next.id << std::endl;
			QRFound = true;
		}
	}

	return QRFound;
}

Corner Detector::subPixelLocation(const cv::Point& point)
{
	// point is one of nonMaximumSuppression corners.

	if (point.x < HALF_PATCH_SIZE ||
		point.y < HALF_PATCH_SIZE ||
		point.x > cmax.cols - HALF_PATCH_SIZE - 1 ||
		point.y > cmax.rows - HALF_PATCH_SIZE - 1)
	{
		return Corner(point.x, point.y);
	}

	int width = cmax.cols, height = cmax.rows;
	cv::Mat patch = cmax(
		cv::Range(std::max(point.y - HALF_PATCH_SIZE, 0), std::min(point.y + HALF_PATCH_SIZE + 1, height)),
		cv::Range(std::max(point.x - HALF_PATCH_SIZE, 0), std::min(point.x + HALF_PATCH_SIZE + 1, width)));
	Eigen::MatrixXf e_patch;
	cv::cv2eigen(patch, e_patch);
	Eigen::Map<Eigen::RowVectorXf> v_patch(e_patch.data(), e_patch.size()); // 创建一个 e_patch 的引用，按列读取转换为行向量。
	auto beta = PATCH_X * v_patch.transpose();
	auto A = beta(0), B = beta(1), C = beta(2), D = beta(3), E = beta(4);
	auto delta = 4 * A * B - E * E;
	if (abs(delta) < 1e-7)
		return Corner(point.x, point.y);

	auto x = -(2 * B * C - D * E) / delta;
	auto y = -(2 * A * D - C * E) / delta;
	if (abs(x) > HALF_PATCH_SIZE || abs(y) > HALF_PATCH_SIZE)
		return Corner(point.x, point.y);

	return Corner(point.x + x, point.y + y);
}

bool Detector::findFirstSecondCorners(const cv::Point& point, CornerTemplate& corner_first, CornerTemplate& corner_second)
{
	corner_first.point = subPixelLocation(point);
	corner_first.width = initWidth;
	corner_second = corner_first;

	PixelType angle11, angle12;
	findEdgeAngles(corner_first.point, angle11, angle12);
	if (abs(angle11) < 1e-7 && abs(angle12) < 1e-7)
		return false;

	PixelType template_angle = (angle11 + angle12 - CV_PI / 2) / 2;
	PixelType corr = calcBolicCorrelation(corner_first.point, WIDTH_MIN/2, template_angle);
	if (corr <= CORR_THRESHOLD)
		return false;

	corner_first.corr = corr;

	/*optimize width and angle. may be useless?*/
	//const int DOUBLE_WIDTH_MIN = 2 * WIDTH_MIN;
	//const PixelType CORR_THRESHOLD_EXT = 0.6f;
	//PixelType corr_x = calcBolicCorrelation(point, DOUBLE_WIDTH_MIN, template_angle);
	//PixelType init_width = corr_x > CORR_THRESHOLD_EXT ? DOUBLE_WIDTH_MIN : WIDTH_MIN;

	std::array<int, 2> comps = { -1, 1}; // search direction.

	//const float k_test = 0.8;
	const int WIDTH_MAX = 100; // TODO: may be changed later.
	for (const int& dir : comps)
	{
		corner_first.width = initWidth;

		while (corner_first.width < WIDTH_MAX)
		{
			Corner next_corner = findNextCorner(corner_first, dir, angle11);

			float width_temp = cv::norm(next_corner - corner_first.point);
			if (width_temp <= WIDTH_MIN) // wrong point, too close. double the width and try again.
			{
				corner_first.width *= 2;
				continue;
			}

			PixelType angle21, angle22;
			findEdgeAngles(next_corner, angle21, angle22);
			PixelType angle_next = (angle21 + angle22 - CV_PI / 2) / 2;
			PixelType corr_test_next = calcBolicCorrelation(next_corner,
				std::max(decltype(width_temp)(WIDTH_MIN/2), (width_temp - WIDTH_MIN)/2), angle_next);
			if (corr_test_next <= CORR_THRESHOLD) // wrong point, noise.
			{
				corner_first.width *= 2;
				continue;
			}

			// useless.
			//auto corr_test_x = calcBolicCorrelation(corner_first.point,
			//	std::max(decltype(width_temp)(WIDTH_MIN), width_temp * k_test), corner_first.angle);
			//if (corr_test_x <= CORR_THRESHOLD_EXT)
			//{
			//	corner_first.width *= 2;
			//	continue;
			//}

			corner_first.width = width_temp;
			corner_first.angle = angle12; // vertical search direction.
			corner_second.point = subPixelLocation(next_corner);
			corner_second.angle = angle22;
			corner_second.width = width_temp;
			corner_second.corr = corr_test_next;
						
			if (corner_first.width < initWidth)
				initWidth = width_temp; // we suppose: only one chessboard.

			return true;
		}
	}

	return false;
}

void Detector::findEdgeAngles(const Corner& point, PixelType& angle1, PixelType& angle2)
{
	int r = 10;
	int width = I_angle.cols, height = I_angle.rows;

	int cu = round(point.x), cv = round(point.y);
	cv::Range v_range = cv::Range(std::max(cv - r, 0), std::min(cv + r + 1, height));
	cv::Range u_range = cv::Range(std::max(cu - r, 0), std::min(cu + r + 1, width));

	edgeOrientation(I_angle(v_range, u_range), I_weight(v_range, u_range), angle1, angle2);
	return;
}

void Detector::edgeOrientation(const cv::Mat& img_angle, const cv::Mat& img_weight, PixelType& angle1, PixelType& angle2)
{
	const int BIN_NUM = 32;
	using Histogram = std::array<PixelType, BIN_NUM>;
	Histogram angle_hist = {};

	/* pair: first--index, second--hist_smoothed(index) */
	using Mode = std::vector<std::pair<int, PixelType>>;

	for (int u = 0; u < img_angle.cols; ++u)
	{
		for (int v = 0; v < img_angle.rows; ++v)
		{
			// img_angle range from: 0 ~ 6.283.
			PixelType val = [](PixelType angle) {
				angle += CV_PI / 2;
				while (angle > CV_PI)
					angle -= CV_PI;
				return angle;
			}(img_angle.ptr<PixelType>(v)[u]);

			auto bin = std::min(static_cast<int>(floor(val / CV_PI * BIN_NUM)), BIN_NUM - 1);

			angle_hist.at(bin) += img_weight.ptr<PixelType>(v)[u];
		}
	}

	auto findModesMeanShift = [&angle_hist, &BIN_NUM](int sigma) {
		Histogram hist_smoothed = {};
		Mode modes;

		// use gaussian distribution to smoothen the histogram.
		auto normpdf = [](PixelType dist, PixelType mu, PixelType sigma) {
			float s = exp(-0.5f * pow((dist - mu) / sigma, 2));
			return s / (sqrt(2 * CV_PI) * sigma);
		};

		for (int i = 0; i < BIN_NUM; ++i)
		{
			for (int j = -2 * sigma; j <= 2 * sigma; ++j)
			{
				auto id = (i + j + BIN_NUM) % BIN_NUM;
				hist_smoothed.at(i) += angle_hist.at(id) * normpdf(j, 0, sigma);
			}
		}

		auto is_all_zeros = [&hist_smoothed]() {
			for (const PixelType& hist : hist_smoothed)
				if (abs(hist - hist_smoothed.front()) >= 1e-5)
					return false;

			return true;
		};
		if (is_all_zeros())
			return modes;

		for (int i = 0; i < BIN_NUM; ++i)
		{
			int j = i;
			while (true)
			{
				PixelType h0 = hist_smoothed.at(j);
				int j1 = (j + 1 + BIN_NUM) % BIN_NUM;
				int j2 = (j - 1 + BIN_NUM) % BIN_NUM;
				PixelType h1 = hist_smoothed.at(j1);
				PixelType h2 = hist_smoothed.at(j2);

				if (h1 >= h0 && h1 >= h2)
					j = j1;
				else if (h2 > h0&& h2 > h1)
					j = j2;
				else
					break;
			}

			// emplace_back j, if not contained.
			auto contains = [&modes](int j) {
				for (const std::pair<int, PixelType>& e : modes)
					if (e.first == j)
						return true;

				return false;
			};
			if (modes.empty() || !contains(j))
			{
				modes.emplace_back(std::make_pair(j, hist_smoothed.at(j)));
			}
		}

		std::sort(modes.begin(), modes.end(),
			[](const std::pair<int, PixelType>& lhs, const std::pair<int, PixelType>& rhs) { return lhs.second > rhs.second; });

		return modes;
	};
	Mode modes = findModesMeanShift(1); // sigma=1.

	if (modes.size() <= 1) {
		angle1 = 0.0;
		angle2 = 0.0;
		return;
	}

	angle1 = modes.at(0).first * CV_PI / BIN_NUM;
	angle2 = modes.at(1).first * CV_PI / BIN_NUM;
	if (abs(angle1-1.57) < abs(angle2-1.57)) // angle range from 0-3.14.
		std::swap(angle1, angle2); // 3.14(horizontal) and 1.57(vertical).

	if (abs(angle2-angle1) <= 0.5f) {
		angle1 = 0.0;
		angle2 = 0.0;
		return;
	}

	return;
}

PixelType Detector::calcBolicCorrelation(const Corner& point, const int& width, const PixelType& theta) const
{
	int rect_rate = 1; // height/width.
	PixelType phi = -theta;

	// hyperbolic tangent function can generate a chessboard corner template.
	auto fun_hyperbolic_tangent_scaled =
		[&theta, &phi](PixelType dx, PixelType dy, PixelType alpha, PixelType beta) { // default: alpha = 1.0, beta = 1.0
		auto fun_hyperbolic_tangent = [&]() {
			PixelType u = -dx * sin(theta) + dy * cos(theta);
			PixelType v = dx * cos(phi) - dy * sin(phi);
			return tanh(alpha * u) * tanh(beta * v);
		};
		return (fun_hyperbolic_tangent() + 1) / 2; // convert to range(0, 1)
	};

	float raw_sum = 0, bolic_sum = 0, cov = 0, var_bolic = 0, var_raw = 0;
	int col_half_width = width, row_half_height = width * rect_rate;
	int count = 0;
	Eigen::Matrix2f rot;
	rot << cos(theta), sin(theta),
		-sin(theta), cos(theta); // cv::Mat 存储顺序与 Eigen 不同。
	for (int x = -col_half_width; x <= col_half_width; ++x)
	{
		for (int y = -row_half_height; y <= row_half_height; ++y)
		{
			Eigen::Vector2f d(x, y);
			Eigen::Vector2f delta = rot * d;
			int input_x = round(point.x + delta.x()), input_y = round(point.y + delta.y());

			if (input_x < 0 || input_x >= gray_image.cols ||
				input_y < 0 || input_y >= gray_image.rows)
			{
				return 0;
			}

			PixelType raw_temp = gray_image.ptr<PixelType>(input_y)[input_x]; // a pointer points to row y, col x.
			PixelType bolic_temp = fun_hyperbolic_tangent_scaled(delta.x(), delta.y(), 1.0, 1.0);

			raw_sum += raw_temp;
			bolic_sum += bolic_temp;
			cov += raw_temp*bolic_temp; // calculate the correlation coefficient.
			var_raw += pow(raw_temp, 2);
			var_bolic += pow(bolic_temp, 2);
			++count;
		}
	}
	float raw_avg = raw_sum / count;
	float bolic_avg = bolic_sum / count;
	cov -= raw_avg*bolic_avg*count; // cov = sum (raw_temp - raw_avg)(bolic_temp - bolic_avg)
	var_raw -= raw_avg*raw_avg*count; // var_raw = sum (raw_temp - raw_avg)(raw_temp - raw_avg)
	var_bolic -= bolic_avg*bolic_avg*count; // var_bolic = sum (bolic_temp - bolic_avg)(bolic_temp - bolic_avg)

	return abs(cov / (sqrt(var_raw * var_bolic)));
}

Corner Detector::findNextCorner(const CornerTemplate& current, const int& dir,const PixelType& searchAngle)
{
	int width = cmax.cols, height = cmax.rows;

	int predict_x = std::min((int)abs(round(current.point.x + dir * current.width * cos(searchAngle))), width);
	int predict_y = std::min((int)abs(round(current.point.y + dir * current.width * sin(searchAngle))), height);

	int side = (int)round(std::max(current.width / 3.0, WIDTH_MIN / 2.0));

	cv::Mat cmax_sub = cmax(
		cv::Range(std::max(predict_y - side, 0), std::min(predict_y + side + 1, height)),
		cv::Range(std::max(predict_x - side, 0), std::min(predict_x + side + 1, width))); // a possible subset.

	cv::Point max_pos;
	cv::minMaxLoc(cmax_sub, nullptr, nullptr, nullptr, &max_pos);

	return Corner(max_pos.x + std::max(predict_x - side, 0), max_pos.y + std::max(predict_y - side, 0));
}

//CornerTemplate Detector::predictPerpNextCorner(const CornerTemplate& current, const int& dir)
//{
//	PixelType searchAngle = current.angle + CV_PI/2;
//	CornerTemplate corner_next(subPixelLocation(findNextCorner(current, dir, searchAngle)), WIDTH_MIN);
//	PixelType angle1, angle2;
//	findEdgeAngles(corner_next.point, angle1, angle2);
//	PixelType angle_next = abs(angle1 - current.angle) < abs(angle2 - current.angle) ? angle1 : angle2;
//	PixelType corr_next = calcBolicCorrelation(corner_next.point,
//		std::max(decltype(current.width)(WIDTH_MIN), current.width - WIDTH_MIN), angle_next);
//	if (corr_next > CORR_THRESHOLD)
//	{
//		corner_next.corr = corr_next;
//		corner_next.angle = angle_next;
//		corner_next.width = cv::norm(corner_next.point - current.point);
//	}
//
//	return corner_next;
//}

CornerTemplate Detector::predictNextCorner(const CornerTemplate& current, const int& dir)
{
	Corner nextTemp = findNextCorner(current, dir, current.angle);
	int xInput = round(nextTemp.x);
	int yInput = nextTemp.y;
	if (cmax.ptr<PixelType>(yInput)[xInput] < 0.2f) //tau, same as above.
	{
		CornerTemplate corner_next(subPixelLocation(nextTemp), current.width);
		corner_next.corr = 0.0;
		return corner_next;
	}

	CornerTemplate corner_next(subPixelLocation(nextTemp), current.width);
	
	PixelType angle1, angle2;
	findEdgeAngles(corner_next.point, angle1, angle2);
	PixelType angle_next = (angle1 + angle2 - CV_PI / 2) / 2;
	PixelType corr_next = calcBolicCorrelation(corner_next.point,
		std::max(decltype(corner_next.width)(WIDTH_MIN/2), (corner_next.width - WIDTH_MIN)/2), angle_next);
	if (corr_next > CORR_THRESHOLD)
	{
		corner_next.corr = corr_next;
		corner_next.angle = angle2;
		corner_next.width = cv::norm(corner_next.point - current.point);
	}

	return corner_next;
}


Eigen::MatrixXf Detector::calcPatchX()
{
	std::vector<int> vec;
	for (int i = -HALF_PATCH_SIZE; i <= HALF_PATCH_SIZE; ++i)
		vec.emplace_back(i);

	int size = 2 * HALF_PATCH_SIZE + 1;
	Eigen::MatrixXf XX = Eigen::MatrixXf(size * size, 6);
	for (int i = 0; i < size * size; ++i)
	{
		int x = vec.at(i / size);
		int y = vec.at(i % size);
		XX(i, 0) = x * x;
		XX(i, 1) = y * y;
		XX(i, 2) = x;
		XX(i, 3) = y;
		XX(i, 4) = x * y;
		XX(i, 5) = 1;
	}

	return (XX.transpose() * XX).inverse()*XX.transpose();
}

cv::Mat Detector::conv2(const cv::Mat &img, const cv::Mat &kernel, const cv::String &mode)
{
	if (mode != "full" && mode != "same" && mode != "valid")
	{
		printf("mode should be: full, same, valid\n");
		return img;
	}

	cv::Mat flip_kernel;
	cv::flip(kernel, flip_kernel, -1); // flip image.
	cv::Mat source = img;
	if (mode == "full")
	{
		source = cv::Mat();
		const int additionalRows = flip_kernel.rows - 1, additionalCols = flip_kernel.cols - 1;
		// add boundary, prepare for convolution.
		cv::copyMakeBorder(img, source, (additionalRows + 1) / 2, additionalRows / 2, (additionalCols + 1) / 2, additionalCols / 2, cv::BORDER_CONSTANT, cv::Scalar(0));
	}
	cv::Point anchor(flip_kernel.cols - flip_kernel.cols / 2 - 1, flip_kernel.rows - flip_kernel.rows / 2 - 1);
	cv::Mat dest;
	cv::filter2D(source, dest, img.depth(), flip_kernel, anchor, 0, cv::BORDER_CONSTANT);

	if (mode == "valid")
	{
		// 抽取部分列
		dest = dest.colRange((flip_kernel.cols - 1) / 2, dest.cols - flip_kernel.cols / 2).rowRange((flip_kernel.rows - 1) / 2, dest.rows - flip_kernel.rows / 2);
	}
	return dest;
}

void Detector::showResult(const cv::String& window_name, const QRsTemplate& QRs, const cv::Mat& image)
{
	bool is_first = true;
	bool is_second = false;
	bool is_third = false;
	for (const QRTemplate& qr : QRs)
	{
		cv::circle(image, qr.corner0, 3, cv::Scalar(0, 255, 0), -1); // green.
		cv::circle(image, qr.corner1, 3, cv::Scalar(255, 0, 0), -1); // blue.
		cv::circle(image, qr.corner2, 3, cv::Scalar(0, 0, 255), -1); // red.
		cv::circle(image, qr.corner3, 3, cv::Scalar(0, 255, 255), -1); // yellow.

		int input_x, input_y;
		// from right to left
		input_x = round(qr.corner3.x * 5 / 2 - qr.corner1.x * 5 / 4 - qr.corner2.x / 4);
		input_y = round(qr.corner3.y * 5 / 2 - qr.corner1.y * 5 / 4 - qr.corner2.y / 4);
		cv::circle(image, { input_x, input_y }, 3, cv::Scalar(255, 255, 0), -1);
		input_x = round(qr.corner3.x * 2 - qr.corner1.x * 5 / 4 + qr.corner2.x / 4);
		input_y = round(qr.corner3.y * 2 - qr.corner1.y * 5 / 4 + qr.corner2.y / 4);
		cv::circle(image, { input_x, input_y }, 3, cv::Scalar(255, 255, 0), -1);
		input_x = round(qr.corner2.x * 2 - qr.corner0.x * 5 / 4 + qr.corner3.x / 4);
		input_y = round(qr.corner2.y * 2 - qr.corner0.y * 5 / 4 + qr.corner3.y / 4);
		cv::circle(image, { input_x, input_y }, 3, cv::Scalar(255, 255, 0), -1);
		input_x = round(qr.corner2.x * 5 / 2 - qr.corner0.x * 5 / 4 - qr.corner3.x / 4);
		input_y = round(qr.corner2.y * 5 / 2 - qr.corner0.y * 5 / 4 - qr.corner3.y / 4);
		cv::circle(image, { input_x, input_y }, 3, cv::Scalar(255, 255, 0), -1);
	}

	//for (const CornerTemplate& c : corners_on_marker)
	//{
	//	cv::circle(image, c.point, 3, cv::Scalar(0, 255, 0), -1); // green.
	//}

	cv::imshow(window_name, image);
	cv::imwrite("CornerDetect.bmp", image);
	cv::waitKey(0);
	// std::cout << "corners size: " << corners.size() << std::endl;
}

int Detector::getQRID(const QRTemplate& QR)
{
	int input_x, input_y;
	PixelType rawCode[4];
	// from right to left
	input_x = round(QR.corner3.x * 5 / 2 - QR.corner1.x * 5 / 4 - QR.corner2.x / 4);
	input_y = round(QR.corner3.y * 5 / 2 - QR.corner1.y * 5 / 4 - QR.corner2.y / 4);
	rawCode[0] = gray_image.ptr<PixelType>(input_y)[input_x];
	input_x = round(QR.corner3.x * 2 - QR.corner1.x * 5 / 4 + QR.corner2.x / 4);
	input_y = round(QR.corner3.y * 2 - QR.corner1.y * 5 / 4 + QR.corner2.y / 4);
	rawCode[1] = gray_image.ptr<PixelType>(input_y)[input_x];
	input_x = round(QR.corner2.x * 2 - QR.corner0.x * 5 / 4 + QR.corner3.x / 4);
	input_y = round(QR.corner2.y * 2 - QR.corner0.y * 5 / 4 + QR.corner3.y / 4);
	rawCode[2] = gray_image.ptr<PixelType>(input_y)[input_x];
	input_x = round(QR.corner2.x * 5 / 2 - QR.corner0.x * 5 / 4 - QR.corner3.x / 4);
	input_y = round(QR.corner2.y * 5 / 2 - QR.corner0.y * 5 / 4 - QR.corner3.y / 4);
	rawCode[3] = gray_image.ptr<PixelType>(input_y)[input_x];

	input_x = round(QR.corner2.x - QR.corner0.x / 2 + QR.corner3.x / 2);
	input_y = round(QR.corner2.y - QR.corner0.y / 2 + QR.corner3.y / 2);
	PixelType rawGT0 = gray_image.ptr<PixelType>(input_y)[input_x]; // white.
	input_x = round(QR.corner0.x / 2 + QR.corner3.x / 2);
	input_y = round(QR.corner0.y / 2 + QR.corner3.y / 2);
	PixelType rawGT1 = gray_image.ptr<PixelType>(input_y)[input_x]; // black.

	int id = 0;
	float colorThresh = (rawGT0-rawGT1)/4;
	for (int i = 0; i < 4; i++) {
		if (abs(rawCode[i] - rawGT0) < colorThresh) {
		}
		else if (abs(rawCode[i] - rawGT1) < colorThresh) {
			id += pow(2, i);
		}
		else {
			return -1;
		}
	}

	return id;

}