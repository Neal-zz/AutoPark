#include "Optimizator.h"

Optimizator::Optimizator(const double& radius, const int& circleNum, const Eigen::Matrix3d& A_cam, const double& threshold, const Corners& corners)
	: radius(radius)
	, circleNum(circleNum)
	, A_cam(A_cam)
	, OPTI_COST_THRESHOLD(threshold)
	, corners(corners)
{
	archL = 2 * EIGEN_PI*radius / circleNum;
}

void Optimizator::process(Eigen::Vector3d& t, double& cost)
{
	const int CORNER_SIZE = corners.size();

	std::vector<Eigen::Vector3d> M_points;
	for (int i = 0; i < CORNER_SIZE; ++i)
	{
		auto angle = i * archL / radius;
		M_points.emplace_back(radius * cos(angle), radius * sin(angle), 1);
	}

	Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
	optimizeExtrinsicPara(M_points, R, t, cost);

	// verify(R, t, M_points);

	///////// calc center on image ///////////
	// Eigen::Matrix3d RR = R;
	// RR.col(2) = t;
	// Eigen::Matrix3d H_left = A_cam * RR;
	// Eigen::Vector3d center = H_left * Eigen::Vector3d(0, 0, 1);
	// Corner center_o(center.x() / center.z(), center.y() / center.z());

	return;
}

void Optimizator::optimizeExtrinsicPara(const std::vector<Eigen::Vector3d>& M_points, Eigen::Matrix3d& Rcam_marker, Eigen::Vector3d& tcam_marker, double& costOut) const
{
	std::map<double, std::pair<Eigen::Matrix3d, Eigen::Vector3d>> opti_record;

	std::array<double, 2> direction = { 0, EIGEN_PI };
	for (const auto& dir : direction)
	{
		auto rotX = [&dir]() {
			Eigen::Matrix3d rot;
			rot << 1, 0, 0,
				0, cos(dir), -sin(dir),
				0, sin(dir), cos(dir);

			return rot;
		};
		Eigen::Matrix3d R = rotX(); // Rcam_world.
		Eigen::Vector3d t = Eigen::Vector3d::Zero(); // tcam_world.

		// relative pose.
		double init_pose[6] = { 1.0, 0, 0, 0, 0, 0 };
		Eigen::Matrix3d R_relative = Eigen::Matrix3d::Zero();
		Eigen::Vector3d t_relative = Eigen::Vector3d::Zero();
		double cost = 0;
		opti(R, t, M_points, init_pose, R_relative, t_relative, cost);

		R = R * R_relative; // Rcam_marker.
		t = t + t_relative; // tcam_marker.

		// judge
		Eigen::Vector3d point_mid = M_points.at(M_points.size() / 2);
		point_mid(2) = 0;
		point_mid = point_mid / point_mid.norm();
		Eigen::Vector3d point_cam = R * point_mid;

		if (point_cam.z() < 0)
		{
			opti_record.insert(std::make_pair(cost, std::make_pair(R, t)));
			if (cost < OPTI_COST_THRESHOLD)
			{
				Rcam_marker = R;
				tcam_marker = t;
				costOut = cost;
				return;
			}
		}
	}

	Rcam_marker = opti_record.begin()->second.first;
	tcam_marker = opti_record.begin()->second.second;
	costOut = opti_record.begin()->first;
	return;
}

struct ReprojectionError
{
	ReprojectionError(const Eigen::Matrix3d& R,
		const Eigen::Vector3d& t,
		const Corner& m,
		const Eigen::Vector3d& M,
		const Eigen::Matrix3d& A_cam)
		: R(R)
		, t(t)
		, m(m)
		, M(M)
		, A_cam(A_cam)
	{
	}
	
	template <typename T>
	bool operator()(const T* const pose, T* residuals) const
	{
		T R_relative[9]; // Rworld_marker.
		T t_relative[3]; // tworld_marker.
		Pose2RT(pose, R_relative, t_relative);

		T R_now[9];
		T t_now[3];
		T RR[9];
		for (int i = 0; i < 9; ++i)
			RR[i] = T(R(i / 3, i % 3)); // Rcam_world.
		T tt[3];
		for (int i = 0; i < 3; ++i)
			tt[i] = T(t(i)); // tcam_world.
		MatMulMat(RR, R_relative, R_now); // Rcam_marker.
		VecAddVec(tt, t_relative, t_now); // tcam_marker.

		T E[9];
		[&R_now, &t_now](T E[9]) {
			for (int i = 0; i < 9; ++i)
				E[i] = R_now[i];

			E[2] = t_now[0];
			E[5] = t_now[1];
			E[8] = t_now[2];
		}(E);

		T H[9];
		T A[9];
		for (int i = 0; i < 9; ++i)
			A[i] = T(A_cam(i / 3, i % 3));
		MatMulMat(A, E, H);

		T MM[3];
		MM[0] = T(M.x());
		MM[1] = T(M.y());
		MM[2] = T(M.z());
		T MMM[3];
		MatMulVec(H, MM, MMM); // Z[u;v;1]=A_cam*[X;Y;Z].

		residuals[0] = T(m.x) - MMM[0] / MMM[2];
		residuals[1] = T(m.y) - MMM[1] / MMM[2];
		return true;
	}

	const Eigen::Matrix3d R; // Rcam_world.
	const Eigen::Vector3d t; // tcam_world.
	const Corner m; // 图像坐标系下的角点坐标
	const Eigen::Vector3d M; // marker坐标系下的角点坐标。[r*cos(angle);r*sin(angle);1]
	const Eigen::Matrix3d A_cam; // camer intrinsic.
};

/*Rcam_world, tcam_world, [r*cos(angle);r*sin(angle);1], poseworld_marker*/
void Optimizator::opti(const Eigen::Matrix3d& R, const Eigen::Vector3d& t, const std::vector<Eigen::Vector3d>& M_points, double pose[6], Eigen::Matrix3d& poseR, Eigen::Vector3d& poset, double& cost) const
{
	ceres::Problem problem;
	for (int i = 0; i < corners.size(); ++i)
	{
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6>(
			new ReprojectionError(R, t, corners.at(i), M_points.at(i), A_cam)),
			nullptr,
			pose);
	}

	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;
	options.num_threads = 8;
	options.function_tolerance = 1e-10;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	// std::cout << summary.BriefReport() << "\n";
	// std::cout << "a= " << pose[0] << std::endl;
	// std::cout << "b= " << pose[1] << std::endl;
	// std::cout << "c= " << pose[2] << std::endl;
	// std::cout << "x= " << pose[3] << std::endl;
	// std::cout << "y= " << pose[4] << std::endl;
	// std::cout << "z= " << pose[5] << std::endl;

	auto rotRPY = [](const double phi, const double theta, const double psi) {
		Eigen::Matrix3d rot;
		rot << cos(phi) * cos(theta), -sin(phi) * cos(psi) + cos(phi) * sin(theta) * sin(psi), sin(phi) * sin(psi) + cos(phi) * sin(theta) * cos(psi),
			sin(phi) * cos(theta), cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(psi), -cos(phi) * sin(psi) + sin(phi) * sin(theta) * cos(psi),
			-sin(theta), cos(theta) * sin(psi), cos(theta) * cos(psi);

		return rot;
	};

	poseR = rotRPY(pose[0], pose[1], pose[2]);
	poset = Eigen::Vector3d(pose[3], pose[4], pose[5]);
	cost = summary.final_cost;
	return;
}


