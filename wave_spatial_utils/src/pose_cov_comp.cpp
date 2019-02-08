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
 * File: ref_ekf.cpp
 * Desc: File containing the impl. for pose composition with uncertainty
 * Auth: Chunshang Li
 *
 * ############################################################################
 */

// #include <mrpt/poses.h>
#include "wave_spatial_utils/pose_cov_comp.hpp"
#include <Eigen/Core>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// using namespace mrpt::poses;
// using namespace mrpt::math;
// using namespace std;

namespace pose_cov_comp {

geometry_msgs::PoseWithCovariance composePose(
  const geometry_msgs::PoseWithCovariance &p1,
  const geometry_msgs::PoseWithCovariance &p2) {
    // use easy to read variable names
    double qr1 = p1.pose.orientation.w, qx1 = p1.pose.orientation.x,
           qy1 = p1.pose.orientation.y, qz1 = p1.pose.orientation.z,
           x1 = p1.pose.position.x, y1 = p1.pose.position.y,
           z1 = p1.pose.position.z;
    double qr2 = p2.pose.orientation.w, qx2 = p2.pose.orientation.x,
           qy2 = p2.pose.orientation.y, qz2 = p2.pose.orientation.z,
           x2 = p2.pose.position.x, y2 = p2.pose.position.y,
           z2 = p2.pose.position.z;

    geometry_msgs::PoseWithCovariance r;  // store all the results

    // Use ROS libraries to transform poses
    // This is the same as implementing Equation (5.5)
    tf2::Transform tf_p1, tf_p2, tf_r;
    tf2::fromMsg(p1.pose, tf_p1);
    tf2::fromMsg(p2.pose, tf_p2);
    tf_r = tf_p1 * tf_p2;
    tf2::toMsg(tf_r, r.pose);

    // compute the covariances
    // p6 = [x y z roll pitch yaw]', transformation using YPR
    // p7 = [x y z qr, qx, qy, qz]', transformation using Quaternion, qr is qw

    // get the covariances from ROS msg
    Matrix6x6 cov_p1(p1.covariance.data()), cov_p2(p2.covariance.data());

    // pR7 is the composed pose in p7 form
    // p17 is the p1 in p7 form, p27 is p2 in p7 form, p16 is p1 in p6 form,
    // p26 is p2 in p6 form
    Vector7 pR7, p17, p27;
    Vector6 p16, p26;
    pR7 << r.pose.position.x, r.pose.position.y, r.pose.position.z,
      r.pose.orientation.w, r.pose.orientation.x, r.pose.orientation.y,
      r.pose.orientation.z;
    p17 << x1, y1, z1, qr1, qx1, qy1, qz1;
    p27 << x2, y2, z2, qr2, qx2, qy2, qz2;

    p16 << x1, y1, z1, 0.0, 0.0, 0.0;
    p26 << x2, y2, z2, 0.0, 0.0, 0.0;

    p16.block<3, 1>(3, 0) = quatToRpy(p17.block<4, 1>(3, 0));
    p26.block<3, 1>(3, 0) = quatToRpy(p27.block<4, 1>(3, 0));

    // Equation (5.3)
    Matrix6x7 jacobian_p7_to_p6 = jacobian_p7_to_p6_wrt_p(pR7);
    Matrix7x7 jacobian_p7_p7_composition =
      jacobian_p7_p7_Composition_wrt_p1(p17, p27);
    Matrix7x6 jacobian_p6_to_p7 = jacobian_p6_to_p7_wrt_p(p16);
    Matrix6x6 dfpc_dp =
      jacobian_p7_to_p6 * jacobian_p7_p7_composition * jacobian_p6_to_p7;

    // Equation (5.4)
    jacobian_p7_p7_composition = jacobian_p7_p7_Composition_wrt_p2(p17, p27);
    jacobian_p6_to_p7 = jacobian_p6_to_p7_wrt_p(p26);
    Matrix6x6 dfpc_dq =
      jacobian_p7_to_p6 * jacobian_p7_p7_composition * jacobian_p6_to_p7;

    // Equation (5.2)
    // putting everything together
    Matrix6x6 final_cov = dfpc_dp * cov_p1 * dfpc_dp.transpose() +
                          dfpc_dq * cov_p2 * dfpc_dq.transpose();

    Eigen::Map<Matrix6x6>(r.covariance.data(), 6, 6) = final_cov;

    return r;
}

/// converting quaternion to rpy
/// Equation (2.9) Equation(2.10)
/// Note v(0) is yaw, v(1) is pitch, v(2) is roll
Vector3 quatToRpy(const Vector4 &q) {
    Vector3 v;

    double qr = q(0), qx = q(1), qy = q(2), qz = q(3);

    double delta = qr * qy - qx * qz;
    double roll, pitch, yaw;

    if (fabs(delta - 0.5) < 1e-6) {
        yaw = -2 * atan2(qx, qr);
        pitch = M_PI / 2;
        roll = 0;

    } else if (fabs(delta + 0.5) < 1e-6) {
        yaw = 2 * atan2(qx, qr);
        pitch = -M_PI / 2;
        roll = 0;
    } else {
        yaw = atan2(2 * (qr * qz + qx * qy), (1 - 2 * (qy * qy + qz * qz)));
        pitch = asin(2 * delta);
        roll = atan2(2 * (qr * qx + qy * qz), (1 - 2 * (qx * qx + qy * qy)));
    }

    v(0) = yaw;
    v(1) = pitch;
    v(2) = roll;

    return v;
}

/// the jacobian of quaternion normalization function
/// quat in the form of [qr, qx, qt, qz]
/// Equation (1.7)
Matrix4x4 jacobian_Quat_Norm_wrt_q(const Vector4 &q) {
    const double &qr = q(0), &qx = q(1), &qy = q(2), &qz = q(3);
    Matrix4x4 m;

    // See Equation (1.7)
    double k = 1 / pow(qr * qr + qx * qx + qy * qy + qz * qz, 1.5);

    // clang-format off
    m << qx * qx + qy * qy + qz * qz, -qr * qx, -qr * qy, -qr * qz,
        -qx * qr, qr * qr + qy * qy + qz * qz, -qx * qy, -qx * qz,
        -qy * qr, -qy * qx, qr * qr + qx * qx + qz * qz, -qy * qz,
        -qz * qr, -qz * qx, -qz * qy, qr * qr + qx * qx + qy * qy;
    // clang-format on

    m = m * k;

    return m;
}

// the jacobian of normalized quaternion to rpy function
// Equation (2.9) to Equation (2.10)
Matrix3x4 jacobian_Quat_Norm_to_Rpy_wrt_q(const Vector4 &q) {
    const double &qr = q(0), &qx = q(1), &qy = q(2), &qz = q(3);

    Matrix3x4 m;

    // Equation (2.9)
    double delta = qr * qy - qx * qz;

    // handle special (rare) cases for when |delta| = 0.5
    if (fabs(delta - 0.5) < 1e-10) {  // delta = 0.5
                                      // clang-format off
        m << (2*qx)/(qr*qr + qx*qx), -(2*qr)/(qr*qr + qx*qx), 0, 0,
                                    0,                     0, 0, 0,
                                    0,                     0, 0, 0;

        // clang-format on

    } else if (fabs(delta + 0.5) < 1e-10) {  // delta = -0.5
                                             // clang-format off
        m << -(2*qx)/(qr*qr + qx*qx), (2*qr)/(qr*qr + qx*qx), 0, 0,
                                     0,                    0, 0, 0,
                                     0,                    0, 0, 0;
                                             // clang-format on
    } else {
        // Equation (2.10)
        // Jacobian obtained symbolically from SymPy by taking the derivative
        // of this below
        //     roll  = [[        2*(qr*qz + qx*qy)/(-2*qy*qy - 2*qz*qz + 1)],
        //     pitch =  [                           asin(2*qr*qy - 2*qx*qz)],
        //     yaw   =  [atan((2*qr*qx + 2*qy*qz)/(-2*qx*qx - 2*qy*qy + 1))]]
        // with respect to qr, qx, qy, qz using SymPy

        // just for convenience
        auto sq = [](double x) { return x * x; };

        // clang-format off
        m <<
            2*qz*(-2*qy*qy - 2*qz*qz + 1)/(sq(2*qr*qz + 2*qx*qy) + sq(-2*qy*qy - 2*qz*qz + 1)),
            2*qy*(-2*qy*qy - 2*qz*qz + 1)/(sq(2*qr*qz + 2*qx*qy) + sq(-2*qy*qy - 2*qz*qz + 1)),
            2*qx*(-2*qy*qy - 2*qz*qz + 1)/(sq(2*qr*qz + 2*qx*qy) + sq(-2*qy*qy - 2*qz*qz + 1)) - 4*qy*(-2*qr*qz - 2*qx*qy)/(sq(2*qr*qz + 2*qx*qy) + sq(-2*qy*qy - 2*qz*qz + 1)),
            2*qr*(-2*qy*qy - 2*qz*qz + 1)/(sq(2*qr*qz + 2*qx*qy) + sq(-2*qy*qy - 2*qz*qz + 1)) - 4*qz*(-2*qr*qz - 2*qx*qy)/(sq(2*qr*qz + 2*qx*qy) + sq(-2*qy*qy - 2*qz*qz + 1)),

            2*qy/sqrt(-sq(2*qr*qy - 2*qx*qz) + 1),
            -2*qz/sqrt(-sq(2*qr*qy - 2*qx*qz) + 1),
            2*qr/sqrt(-sq(2*qr*qy - 2*qx*qz) + 1),
            -2*qx/sqrt(-sq(2*qr*qy - 2*qx*qz) + 1),

            2*qx*(-2*qx*qx - 2*qy*qy + 1)/(sq(2*qr*qx + 2*qy*qz) + sq(-2*qx*qx - 2*qy*qy + 1)),
            2*qr*(-2*qx*qx - 2*qy*qy + 1)/(sq(2*qr*qx + 2*qy*qz) + sq(-2*qx*qx - 2*qy*qy + 1)) - 4*qx*(-2*qr*qx - 2*qy*qz)/(sq(2*qr*qx + 2*qy*qz) + sq(-2*qx*qx - 2*qy*qy + 1)),
            -4*qy*(-2*qr*qx - 2*qy*qz)/(sq(2*qr*qx + 2*qy*qz) + sq(-2*qx*qx - 2*qy*qy + 1)) + 2*qz*(-2*qx*qx - 2*qy*qy + 1)/(sq(2*qr*qx + 2*qy*qz) + sq(-2*qx*qx - 2*qy*qy + 1)),
            2*qy*(-2*qx*qx - 2*qy*qy + 1)/(sq(2*qr*qx + 2*qy*qz) + sq(-2*qx*qx - 2*qy*qy + 1));

        // clang-format on
    }

    return m;
}

/// the jacobian of p7 to p6 conversion
/// Equation (2.12)
Matrix6x7 jacobian_p7_to_p6_wrt_p(const Vector7 &p) {
    Matrix6x7 r = Matrix6x7::Zero();

    // Equation (2.12)
    // bottom left 3x3 is zero
    // top right 3x4 is zero
    // top left 3x3 is identity
    r(0, 0) = 1;
    r(1, 1) = 1;
    r(2, 2) = 1;

    // This function takes the un-normalized quaternion
    Matrix4x4 jacobian_quat_norm =
      jacobian_Quat_Norm_wrt_q(p.block<4, 1>(3, 0));

    Matrix3x4 jacobian_quat_norm_to_rpy =
      jacobian_Quat_Norm_to_Rpy_wrt_q(p.block<4, 1>(3, 0));

    r.block<3, 4>(3, 3) = jacobian_quat_norm_to_rpy * jacobian_quat_norm;

    return r;
}

// jacobian of composing a point to a p7
// Equation (3.8)
Matrix3x7 jacobian_p7_Point_Composition_wrt_p(const Vector7 &p,
                                              const Vector3 &a) {
    Matrix3x7 m = Matrix3x7::Zero();

    // The 3x3 on the top left is the identity matrix
    m.block<3, 3>(0, 0) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

    // Equation (3.9)
    double ax = a(0), ay = a(1), az = a(2);
    double qr = p(3), qx = p(4), qy = p(5), qz = p(6);

    // clang-format off
    m.block<3, 4>(0, 3) <<
        -qz*ay+qy*az, qy*ay+qz*az, -2*qy*ax+qx*ay+qr*az, -2*qz*ax-qr*ay+qx*az,
        qz*ax-qx*az, qy*ax-2*qx*ay-qr*az, qx*ax+qz*az, qr*ax-2*qz*ay+qy*az,
        -qy*ax+qx*ay, qz*ax+qr*ay-2*qx*az, -qr*ax+qz*ay-2*qy*az, qx*ax+qy*ay;
    // clang-format on

    m.block<3, 4>(0, 3) *= 2;

    // applying jacobian normalization
    Matrix4x4 jacobian_quat_norm =
      jacobian_Quat_Norm_wrt_q(p.block<4, 1>(3, 0));
    m.block<3, 4>(0, 3) *= jacobian_quat_norm;

    return m;
}

// jacobian of the composition of p7 poses
// Equation (5.8)
Matrix7x7 jacobian_p7_p7_Composition_wrt_p1(const Vector7 &p1,
                                            const Vector7 &p2) {
    Matrix7x7 m = Matrix7x7::Zero();

    Vector4 q2 = p2.block<4, 1>(3, 0);
    double qr2 = q2(0), qx2 = q2(1), qy2 = q2(2), qz2 = q2(3);

    // clang-format off
    m.block<4, 4>(3, 3) << qr2, -qx2, -qy2, -qz2,
                           qx2, qr2, qz2, -qy2,
                           qy2, -qz2, qr2, qx2,
                           qz2, qy2, -qx2, qr2;
    // clang-format on

    // Note: this quaternion normalization jacobian matrix is not present in
    // the book's formulation, but it should be there if a jacobian
    // normalization is performed on the input
    Matrix4x4 jacobian_quat_norm =
      jacobian_Quat_Norm_wrt_q(p1.block<4, 1>(3, 0));
    m.block<4, 4>(3, 3) *= jacobian_quat_norm;

    m.block<3, 7>(0, 0) =
      jacobian_p7_Point_Composition_wrt_p(p1, p2.block<3, 1>(0, 0));

    return m;
}

// jacobian of composing a point to a p7
// Equation (3.10)
Matrix3x3 jacobian_p7_Point_Composition_wrt_a(const Vector7 &p,
                                              const Vector3 &a) {
    Matrix3x3 m = Matrix3x3::Zero();

    double qr = p(3), qx = p(4), qy = p(5), qz = p(6);

    // clang-format off
    m << 0.5 - qy*qy - qz*qz , qx*qy - qr*qz, qr*qy + qx*qz,
         qr*qz + qx*qy, 0.5 - qx*qx - qz*qz, qy*qz - qr*qx,
         qx*qz - qr*qy, qr*qx + qy*qz, 0.5 - qx*qx - qy*qy;
    // clang-format on

    m *= 2;

    return m;
}

// jacobian of the composition of p7 poses
// Equation (5.9)
Matrix7x7 jacobian_p7_p7_Composition_wrt_p2(const Vector7 &p1,
                                            const Vector7 &p2) {
    Matrix7x7 m = Matrix7x7::Zero();

    Vector4 q1 = p1.block<4, 1>(3, 0);
    double qr1 = q1(0), qx1 = q1(1), qy1 = q1(2), qz1 = q1(3);

    // clang-format off
    m.block<4, 4>(3, 3) << qr1, -qx1, -qy1, -qz1,
                           qx1, qr1, -qz1, qy1,
                           qy1, qz1, qr1, -qx1,
                           qz1, -qy1, qx1, qr1;
    // clang-format on

    // Note: this quaternion normalization jacobian matrix is not present in
    // the book's formulation, but it should be there if a jacobian
    // normalization is performed on the input
    Matrix4x4 jacobian_quat_norm =
      jacobian_Quat_Norm_wrt_q(p2.block<4, 1>(3, 0));
    m.block<4, 4>(3, 3) *= jacobian_quat_norm;

    m.block<3, 3>(0, 0) =
      jacobian_p7_Point_Composition_wrt_a(p1, p2.block<3, 1>(0, 0));

    return m;
}

// jacobian of converting a p6 to a p7
// Equation (2.8)
Matrix7x6 jacobian_p6_to_p7_wrt_p(const Vector6 &p) {
    Matrix7x6 m = Matrix7x6::Zero();

    // top left corner is a identity matrix
    // the bottom left 4x3 and top right 3x3 is zero
    m.block<3, 3>(0, 0) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

    double ccc, ccs, csc, scc, ssc, sss, scs, css;
    double roll = p(5), pitch = p(4), yaw = p(3);
    ccc = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2);
    ccs = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
    csc = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
    scs = sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
    css = cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    scc = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2);
    ssc = sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
    sss = sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);

    // clang-format off
    m.block<4, 3>(3, 3) <<
        (ssc - ccs) / 2.0, (scs - csc) / 2.0, (css - scc) / 2.0,
        -(csc + scs) / 2.0, -(ssc + ccs) / 2.0, (ccc + sss) / 2.0,
        (scc - css) / 2.0, (ccc - sss) / 2.0, (ccs - ssc) / 2.0,
        (ccc + sss) / 2.0, -(css + scc) / 2.0, -(csc + scs) / 2.0;
    // clang-format on

    return m;
}
}
