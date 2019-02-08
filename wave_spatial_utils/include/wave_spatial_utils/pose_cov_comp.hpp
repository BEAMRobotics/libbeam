/* Copyright (c) 2017-2018, Waterloo Autonomous Vehicles Laboratory (WAVELab),
 * Waterloo Intelligent Systems Engineering Lab (WISELab),
 * University of Waterloo.
 *
 * Refer to the accompanying LICENSE file for license information.
 *
 * ############################################################################
 ******************************************************************************
 |                                                                            |
 |                         /\/\__/\_/\      /\_/\__/\/\                       |
 |                         \          \____/          /                       |
 |                          '----________________----'                        |
 |                              /                \                            |
 |                            O/_____/_______/____\O                          |
 |                            /____________________\                          |
 |                           /    (#UNIVERSITY#)    \                         |
 |                           |[**](#OFWATERLOO#)[**]|                         |
 |                           \______________________/                         |
 |                            |_""__|_,----,_|__""_|                          |
 |                            ! !                ! !                          |
 |                            '-'                '-'                          |
 |       __    _   _  _____  ___  __  _  ___  _    _  ___  ___   ____  ____   |
 |      /  \  | | | ||_   _|/ _ \|  \| |/ _ \| \  / |/ _ \/ _ \ /     |       |
 |     / /\ \ | |_| |  | |  ||_||| |\  |||_|||  \/  |||_||||_|| \===\ |====   |
 |    /_/  \_\|_____|  |_|  \___/|_| \_|\___/|_|\/|_|\___/\___/ ____/ |____   |
 |                                                                            |
 ******************************************************************************
 * ############################################################################
 *
 * File: ref_ekf.hpp
 * Desc: File containing the function for pose composition with uncertainty
 * Auth: Chunshang Li
 *
 * ############################################################################
 */

#ifndef REF_EKF_POSE_COV_COMP_HPP_
#define REF_EKF_POSE_COV_COMP_HPP_

#include <Eigen/Core>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace pose_cov_comp {

typedef Eigen::Matrix<double, 7, 1> Vector7;
typedef Eigen::Matrix<double, 6, 1> Vector6;
typedef Eigen::Matrix<double, 4, 1> Vector4;
typedef Eigen::Matrix<double, 3, 1> Vector3;
typedef Eigen::Matrix<double, 6, 6, Eigen::RowMajor> Matrix6x6;
typedef Eigen::Matrix<double, 6, 7, Eigen::RowMajor> Matrix6x7;
typedef Eigen::Matrix<double, 7, 6, Eigen::RowMajor> Matrix7x6;
typedef Eigen::Matrix<double, 4, 4, Eigen::RowMajor> Matrix4x4;
typedef Eigen::Matrix<double, 3, 4, Eigen::RowMajor> Matrix3x4;
typedef Eigen::Matrix<double, 7, 7, Eigen::RowMajor> Matrix7x7;
typedef Eigen::Matrix<double, 3, 7, Eigen::RowMajor> Matrix3x7;
typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Matrix3x3;

geometry_msgs::PoseWithCovariance composePose(
  const geometry_msgs::PoseWithCovariance &p1,
  const geometry_msgs::PoseWithCovariance &p2);

/// converting quaternion to rpy
/// Equation (2.9) Equation(2.10)
Vector3 quatToRpy(const Vector4 &q);

/// the jacobian of quaternion normalization function
/// quat in the form of [qr, qx, qt, qz]
/// Equation (1.7)
Matrix4x4 jacobian_Quat_Norm_wrt_q(const Vector4 &q);

// the jacobian of normalized quaternion to rpy function
// Equation (2.9) to Equation (2.10)
Matrix3x4 jacobian_Quat_Norm_to_Rpy_wrt_q(const Vector4 &q);

/// the jocobian of p7 to p6 conversion
/// Equation (2.12)
Matrix6x7 jacobian_p7_to_p6_wrt_p(const Vector7 &p);

/// jacobian of composing a point to a p7
/// Equation (3.8)
Matrix3x7 jacobian_p7_Point_Composition_wrt_p(const Vector7 &p,
                                              const Vector3 &a);
/// jacobian of the composition of p7 poses
/// Equation (5.8)
Matrix7x7 jacobian_p7_p7_Composition_wrt_p1(const Vector7 &p1,
                                            const Vector7 &p2);

/// jacobian of composing a point to a p7
/// Equation (3.10)
Matrix3x3 jacobian_p7_Point_Composition_wrt_a(const Vector7 &p,
                                              const Vector3 &a);

/// jacobian of the composition of p7 poses
/// Equation (5.9)
Matrix7x7 jacobian_p7_p7_Composition_wrt_p2(const Vector7 &p1,
                                            const Vector7 &p2);

/// jacobian of converting a p6 to a p7
/// Equation (2.8)
Matrix7x6 jacobian_p6_to_p7_wrt_p(const Vector6 &p);
}  // namespace pose_cov_comp


#endif
