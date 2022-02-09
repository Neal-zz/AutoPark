#pragma once

#include <Eigen/Dense>
#include <ceres/ceres.h>

#include "Detector.h"

//double OPTI_COST_THRESHOLD;

template <typename T>
inline T DotProduct(const T x[3], const T y[3])
{
	return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2]);
}

template <typename T>
inline void RotPRY(const T* pose, T mat[9])
{
	const T phi = pose[0];
	const T theta = pose[1];
	const T psi = pose[2];

	mat[0] = cos(phi) * cos(theta); // 按行存储。
	mat[1] = -sin(phi) * cos(psi) + cos(phi) * sin(theta) * sin(psi);
	mat[2] = sin(phi) * sin(psi) + cos(phi) * sin(theta) * cos(psi);
	mat[3] = sin(phi) * cos(theta);
	mat[4] = cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(psi);
	mat[5] = -cos(phi) * sin(psi) + sin(phi) * sin(theta) * cos(psi);
	mat[6] = -sin(theta);
	mat[7] = cos(theta) * sin(psi);
	mat[8] = cos(theta) * cos(psi);
}

// 传递数组给一个函数，数组类型自动转换为指针类型，因而传的实际是地址。
// 在长度未知的情况下，用 pose 的方式传递；长度已知的情况下用 R 和 t 的方式传递。
template <typename T>
inline void Pose2RT(const T* pose, T R[9], T t[3])
{
	RotPRY(pose, R);
	t[0] = pose[3];
	t[1] = pose[4];
	t[2] = pose[5];
}

template <typename T>
inline void MatMulMat(const T A[9], const T B[9], T result[9])
{
	T A_row_1[3] = { A[0], A[1], A[2] };
	T A_row_2[3] = { A[3], A[4], A[5] };
	T A_row_3[3] = { A[6], A[7], A[8] };

	T B_col_1[3] = { B[0], B[3], B[6] };
	T B_col_2[3] = { B[1], B[4], B[7] };
	T B_col_3[3] = { B[2], B[5], B[8] };

	result[0] = DotProduct(A_row_1, B_col_1);
	result[1] = DotProduct(A_row_1, B_col_2);
	result[2] = DotProduct(A_row_1, B_col_3);

	result[3] = DotProduct(A_row_2, B_col_1);
	result[4] = DotProduct(A_row_2, B_col_2);
	result[5] = DotProduct(A_row_2, B_col_3);

	result[6] = DotProduct(A_row_3, B_col_1);
	result[7] = DotProduct(A_row_3, B_col_2);
	result[8] = DotProduct(A_row_3, B_col_3);
}

template <typename T>
inline void MatMulVec(const T A[9], const T B[3], T result[3])
{
	T A_row_1[3] = { A[0], A[1], A[2] };
	T A_row_2[3] = { A[3], A[4], A[5] };
	T A_row_3[3] = { A[6], A[7], A[8] };

	result[0] = DotProduct(A_row_1, B);
	result[1] = DotProduct(A_row_2, B);
	result[2] = DotProduct(A_row_3, B);
}

template <typename T>
inline void VecAddVec(const T A[3], const T B[3], T result[3])
{
	result[0] = A[0] + B[0];
	result[1] = A[1] + B[1];
	result[2] = A[2] + B[2];
}

class Optimizator
{
public:
	Optimizator(const double& radius, const int& circleNum, const Eigen::Matrix3d& A_cam, const double& threshold, const Corners& corners);

	/*t, cost*/
	void process(Eigen::Vector3d& t, double& cost);

private:
	/*Rcam_marker, tcam_marker, cost*/
	void optimizeExtrinsicPara(const std::vector<Eigen::Vector3d>& M_points, Eigen::Matrix3d& Rcam_marker, Eigen::Vector3d& tcam_marker, double& cost) const;

	/*poseR, poset, cost */
	void opti(const Eigen::Matrix3d& R, const Eigen::Vector3d& t, const std::vector<Eigen::Vector3d>& M_points, double pose[6], Eigen::Matrix3d& poseR, Eigen::Vector3d& poset, double& cost) const;

	double radius; // marker 半径。
	int circleNum; // 一圈上角点个数。
	double archL; // 弧长。=2pi*radius/circleNum.

	Eigen::Matrix3d A_cam; // camera intrinsic.
	double OPTI_COST_THRESHOLD;

	Corners corners;
};