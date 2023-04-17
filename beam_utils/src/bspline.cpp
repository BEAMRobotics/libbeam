/*
 * ADAPTED FOR BEAM FROM:
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2022 Patrick Geneva
 * Copyright (C) 2018-2022 Guoquan Huang
 * Copyright (C) 2018-2022 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <beam_utils/bspline.h>
#include <beam_utils/log.h>

namespace beam {

void BsplineSE3::feed_trajectory(const std::vector<beam::Pose>& trajectory) {
  std::vector<Eigen::VectorXd> traj_points;
  for (const auto& p : trajectory) {
    traj_points.push_back(pose_to_vectorXd(p));
  }
  feed_trajectory(traj_points);
}

void BsplineSE3::feed_trajectory(
    const std::map<int64_t, beam::Pose>& trajectory) {
  std::vector<Eigen::VectorXd> traj_points;
  for (const auto& [t, p] : trajectory) {
    traj_points.push_back(pose_to_vectorXd(p));
  }
  feed_trajectory(traj_points);
}

void BsplineSE3::feed_trajectory(
    const std::vector<Eigen::VectorXd>& traj_points) {
  if (traj_points.size() < 4) {
    BEAM_ERROR("spline trajectory must contain at least 4 poses");
    return;
  }

  // Find the average frequency to use as our uniform timesteps
  double sumdt = 0;
  for (size_t i = 0; i < traj_points.size() - 1; i++) {
    sumdt += traj_points.at(i + 1)(0) - traj_points.at(i)(0);
  }
  dt = sumdt / (traj_points.size() - 1);
  dt = (dt < 0.05) ? 0.05 : dt;
  BEAM_DEBUG("[B-SPLINE]: control point dt = {} (original dt of {})", dt,
             sumdt / (traj_points.size() - 1));

  // convert all our trajectory points into SE(3) matrices
  // we are given [timestamp, p_IinG, q_GtoI]
  AlignedEigenMat4d trajectory_points;
  for (size_t i = 0; i < traj_points.size(); i++) {
    Eigen::Matrix4d T_IinG = Eigen::Matrix4d::Identity();
    T_IinG.block(0, 0, 3, 3) =
        Quat2Rot(traj_points.at(i).block(4, 0, 4, 1)).transpose();
    T_IinG.block(0, 3, 3, 1) = traj_points.at(i).block(1, 0, 3, 1);
    trajectory_points.insert({traj_points.at(i)(0), T_IinG});
  }

  // Get the oldest timestamp
  double timestamp_min = INFINITY;
  double timestamp_max = -INFINITY;
  for (const auto& pose : trajectory_points) {
    if (pose.first <= timestamp_min) { timestamp_min = pose.first; }
    if (pose.first >= timestamp_max) { timestamp_max = pose.first; }
  }
  BEAM_DEBUG("[B-SPLINE]: trajectory start time = {}", timestamp_min);
  BEAM_DEBUG("[B-SPLINE]: trajectory end time = {}", timestamp_max);

  // then create spline control points
  double timestamp_curr = timestamp_min;
  while (true) {
    // Get bounding posed for the current time
    double t0, t1;
    Eigen::Matrix4d pose0, pose1;
    bool success = find_bounding_poses(timestamp_curr, trajectory_points, t0,
                                       pose0, t1, pose1);

    if (!success) {
      success = get_2_boundary_points(timestamp_curr, trajectory_points, t0,
                                      pose0, t1, pose1);
    }

    // If we didn't find a bounding pose, then that means we are at outside of
    // the dataset Thus break out of this loop since we have created our max
    // number of control points
    if (!success) break;

    // Linear interpolation and append to our control points
    double lambda = (timestamp_curr - t0) / (t1 - t0);
    Eigen::Matrix4d pose_interp =
        ExpSe3(lambda * LogSe3(pose1 * InvSe3(pose0))) * pose0;
    control_points.insert({timestamp_curr, pose_interp});
    timestamp_curr += dt;
  }

  // The start time of the system is two dt in since we need at least two older
  // control points
  timestamp_start = timestamp_min + 2 * dt;
  BEAM_DEBUG("[B-SPLINE]: start trajectory time of {}", timestamp_start);
}

bool BsplineSE3::get_pose(double timestamp, Eigen::Matrix4d& T_G_I) {
  if (control_points.size() < 4) {
    BEAM_ERROR("spline not properly initialized, cannot get pose");
    return false;
  }

  Eigen::Matrix3d R_GtoI;
  Eigen::Vector3d p_IinG;

  if (!get_pose(timestamp, R_GtoI, p_IinG)) { return false; }

  Eigen::Matrix4d T;
  T_G_I.block(0, 0, 3, 3) = R_GtoI;
  T_G_I.block(0, 3, 3, 1) = p_IinG;
  return true;
}

bool BsplineSE3::get_pose_or_extrapolate(double timestamp,
                                         Eigen::Matrix4d& T_G_I) {
  if (control_points.size() < 4) {
    BEAM_ERROR("spline not properly initialized, cannot get pose");
    return false;
  }

  Eigen::Matrix3d R_GtoI;
  Eigen::Vector3d p_IinG;

  if (get_pose(timestamp, R_GtoI, p_IinG)) {
    Eigen::Matrix4d T;
    T_G_I.block(0, 0, 3, 3) = R_GtoI;
    T_G_I.block(0, 3, 3, 1) = p_IinG;
    return true;
  }

  return extrapolate(timestamp, T_G_I);
}

bool BsplineSE3::extrapolate(double timestamp,
                             Eigen::Matrix4d& T_FIXED_BASELINK) {
  if (control_points.size() < 4) {
    BEAM_ERROR("spline not properly initialized, cannot get pose");
    return false;
  }

  T_FIXED_BASELINK.setIdentity();
  Eigen::Matrix3d R_GtoI;
  Eigen::Vector3d p_IinG;
  Eigen::Vector3d w_IinI;
  Eigen::Vector3d v_IinG;
  double end_time = control_points.rbegin()->first;

  if (!get_velocity(end_time, R_GtoI, p_IinG, w_IinI, v_IinG)) { return false; }

  // extrapolate
  double dt = timestamp - end_time;
  const Eigen::Matrix3d& R_FIXED_BASELINKLAST = R_GtoI;
  const Eigen::Vector3d& t_FIXED_BASELINKLAST = p_IinG;
  Eigen::Matrix3d R_BaselinkLast_BaselinkQuery =
      (beam::ExpSo3(w_IinI * dt)).transpose();
  Eigen::Vector3d t_BaselinkLast_BaselinkQuery_in_fixed = v_IinG * dt;

  T_FIXED_BASELINK.block(0, 0, 3, 3) =
      R_FIXED_BASELINKLAST * R_BaselinkLast_BaselinkQuery;
  T_FIXED_BASELINK.block(0, 3, 3, 1) =
      t_FIXED_BASELINKLAST + t_BaselinkLast_BaselinkQuery_in_fixed;
  return true;
}

// Loop through this line (timestamp(s) tx ty tz qx qy qz qw)
Eigen::VectorXd BsplineSE3::pose_to_vectorXd(const Pose& pose) {
  Eigen::Matrix3d R = pose.T_FIXED_MOVING.block(0, 0, 3, 3).transpose();
  Eigen::Vector3d t = pose.T_FIXED_MOVING.block(0, 3, 3, 1);
  Eigen::Quaterniond q(R);
  Eigen::VectorXd v(8);
  v(0) = static_cast<double>(pose.timestampInNs * 1e-9);
  v.block(1, 0, 3, 1) = t;
  v(4) = q.x();
  v(5) = q.y();
  v(6) = q.z();
  v(7) = q.w();
  return v;
}

bool BsplineSE3::get_pose(double timestamp, Eigen::Matrix3d& R_GtoI,
                          Eigen::Vector3d& p_IinG) {
  if (control_points.size() < 4) {
    BEAM_ERROR("spline not properly initialized, cannot get pose");
    return false;
  }

  // Get the bounding poses for the desired timestamp
  double t0, t1, t2, t3;
  Eigen::Matrix4d pose0, pose1, pose2, pose3;
  bool success = find_bounding_control_points(
      timestamp, control_points, t0, pose0, t1, pose1, t2, pose2, t3, pose3);

  // Return failure if we can't get bounding poses
  if (!success) {
    R_GtoI.setIdentity();
    p_IinG.setZero();
    return false;
  }

  // Our De Boor-Cox matrix scalars
  double DT = (t2 - t1);
  double u = (timestamp - t1) / DT;
  double b0 = 1.0 / 6.0 * (5 + 3 * u - 3 * u * u + u * u * u);
  double b1 = 1.0 / 6.0 * (1 + 3 * u + 3 * u * u - 2 * u * u * u);
  double b2 = 1.0 / 6.0 * (u * u * u);

  // Calculate interpolated poses
  Eigen::Matrix4d A0 = ExpSe3(b0 * LogSe3(InvSe3(pose0) * pose1));
  Eigen::Matrix4d A1 = ExpSe3(b1 * LogSe3(InvSe3(pose1) * pose2));
  Eigen::Matrix4d A2 = ExpSe3(b2 * LogSe3(InvSe3(pose2) * pose3));

  // Finally get the interpolated pose
  Eigen::Matrix4d pose_interp = pose0 * A0 * A1 * A2;
  R_GtoI = pose_interp.block(0, 0, 3, 3).transpose();
  p_IinG = pose_interp.block(0, 3, 3, 1);
  return true;
}

bool BsplineSE3::get_velocity(double timestamp, Eigen::Matrix3d& R_GtoI,
                              Eigen::Vector3d& p_IinG, Eigen::Vector3d& w_IinI,
                              Eigen::Vector3d& v_IinG) {
  if (control_points.size() < 4) {
    BEAM_ERROR("spline not properly initialized, cannot get pose");
    return false;
  }

  // Get the bounding poses for the desired timestamp
  double t0, t1, t2, t3;
  Eigen::Matrix4d pose0, pose1, pose2, pose3;
  bool success = find_bounding_control_points(
      timestamp, control_points, t0, pose0, t1, pose1, t2, pose2, t3, pose3);

  // Return failure if we can't get bounding poses
  if (!success) {
    w_IinI.setZero();
    v_IinG.setZero();
    return false;
  }

  // Our De Boor-Cox matrix scalars
  double DT = (t2 - t1);
  double u = (timestamp - t1) / DT;
  double b0 = 1.0 / 6.0 * (5 + 3 * u - 3 * u * u + u * u * u);
  double b1 = 1.0 / 6.0 * (1 + 3 * u + 3 * u * u - 2 * u * u * u);
  double b2 = 1.0 / 6.0 * (u * u * u);
  double b0dot = 1.0 / (6.0 * DT) * (3 - 6 * u + 3 * u * u);
  double b1dot = 1.0 / (6.0 * DT) * (3 + 6 * u - 6 * u * u);
  double b2dot = 1.0 / (6.0 * DT) * (3 * u * u);

  // Cache some values we use alot
  Eigen::Matrix<double, 6, 1> omega_10 = LogSe3(InvSe3(pose0) * pose1);
  Eigen::Matrix<double, 6, 1> omega_21 = LogSe3(InvSe3(pose1) * pose2);
  Eigen::Matrix<double, 6, 1> omega_32 = LogSe3(InvSe3(pose2) * pose3);

  // Calculate interpolated poses
  Eigen::Matrix4d A0 = ExpSe3(b0 * omega_10);
  Eigen::Matrix4d A1 = ExpSe3(b1 * omega_21);
  Eigen::Matrix4d A2 = ExpSe3(b2 * omega_32);
  Eigen::Matrix4d A0dot = b0dot * HatSe3(omega_10) * A0;
  Eigen::Matrix4d A1dot = b1dot * HatSe3(omega_21) * A1;
  Eigen::Matrix4d A2dot = b2dot * HatSe3(omega_32) * A2;

  // Get the interpolated pose
  Eigen::Matrix4d pose_interp = pose0 * A0 * A1 * A2;
  R_GtoI = pose_interp.block(0, 0, 3, 3).transpose();
  p_IinG = pose_interp.block(0, 3, 3, 1);

  // Finally get the interpolated velocities
  // NOTE: Rdot = R*skew(omega) => R^T*Rdot = skew(omega)
  Eigen::Matrix4d vel_interp =
      pose0 * (A0dot * A1 * A2 + A0 * A1dot * A2 + A0 * A1 * A2dot);
  w_IinI = Vee(pose_interp.block(0, 0, 3, 3).transpose() *
               vel_interp.block(0, 0, 3, 3));
  v_IinG = vel_interp.block(0, 3, 3, 1);
  return true;
}

bool BsplineSE3::get_acceleration(double timestamp, Eigen::Matrix3d& R_GtoI,
                                  Eigen::Vector3d& p_IinG,
                                  Eigen::Vector3d& w_IinI,
                                  Eigen::Vector3d& v_IinG,
                                  Eigen::Vector3d& alpha_IinI,
                                  Eigen::Vector3d& a_IinG) {
  if (control_points.size() < 4) {
    BEAM_ERROR("spline not properly initialized, cannot get pose");
    return false;
  }

  // Get the bounding poses for the desired timestamp
  double t0, t1, t2, t3;
  Eigen::Matrix4d pose0, pose1, pose2, pose3;
  bool success = find_bounding_control_points(
      timestamp, control_points, t0, pose0, t1, pose1, t2, pose2, t3, pose3);

  // Return failure if we can't get bounding poses
  if (!success) {
    alpha_IinI.setZero();
    a_IinG.setZero();
    return false;
  }

  // Our De Boor-Cox matrix scalars
  double DT = (t2 - t1);
  double u = (timestamp - t1) / DT;
  double b0 = 1.0 / 6.0 * (5 + 3 * u - 3 * u * u + u * u * u);
  double b1 = 1.0 / 6.0 * (1 + 3 * u + 3 * u * u - 2 * u * u * u);
  double b2 = 1.0 / 6.0 * (u * u * u);
  double b0dot = 1.0 / (6.0 * DT) * (3 - 6 * u + 3 * u * u);
  double b1dot = 1.0 / (6.0 * DT) * (3 + 6 * u - 6 * u * u);
  double b2dot = 1.0 / (6.0 * DT) * (3 * u * u);
  double b0dotdot = 1.0 / (6.0 * DT * DT) * (-6 + 6 * u);
  double b1dotdot = 1.0 / (6.0 * DT * DT) * (6 - 12 * u);
  double b2dotdot = 1.0 / (6.0 * DT * DT) * (6 * u);

  // Cache some values we use alot
  Eigen::Matrix<double, 6, 1> omega_10 = LogSe3(InvSe3(pose0) * pose1);
  Eigen::Matrix<double, 6, 1> omega_21 = LogSe3(InvSe3(pose1) * pose2);
  Eigen::Matrix<double, 6, 1> omega_32 = LogSe3(InvSe3(pose2) * pose3);
  Eigen::Matrix4d omega_10_hat = HatSe3(omega_10);
  Eigen::Matrix4d omega_21_hat = HatSe3(omega_21);
  Eigen::Matrix4d omega_32_hat = HatSe3(omega_32);

  // Calculate interpolated poses
  Eigen::Matrix4d A0 = ExpSe3(b0 * omega_10);
  Eigen::Matrix4d A1 = ExpSe3(b1 * omega_21);
  Eigen::Matrix4d A2 = ExpSe3(b2 * omega_32);
  Eigen::Matrix4d A0dot = b0dot * omega_10_hat * A0;
  Eigen::Matrix4d A1dot = b1dot * omega_21_hat * A1;
  Eigen::Matrix4d A2dot = b2dot * omega_32_hat * A2;
  Eigen::Matrix4d A0dotdot =
      b0dot * omega_10_hat * A0dot + b0dotdot * omega_10_hat * A0;
  Eigen::Matrix4d A1dotdot =
      b1dot * omega_21_hat * A1dot + b1dotdot * omega_21_hat * A1;
  Eigen::Matrix4d A2dotdot =
      b2dot * omega_32_hat * A2dot + b2dotdot * omega_32_hat * A2;

  // Get the interpolated pose
  Eigen::Matrix4d pose_interp = pose0 * A0 * A1 * A2;
  R_GtoI = pose_interp.block(0, 0, 3, 3).transpose();
  p_IinG = pose_interp.block(0, 3, 3, 1);

  // Get the interpolated velocities
  // NOTE: Rdot = R*skew(omega) => R^T*Rdot = skew(omega)
  Eigen::Matrix4d vel_interp =
      pose0 * (A0dot * A1 * A2 + A0 * A1dot * A2 + A0 * A1 * A2dot);
  w_IinI = Vee(pose_interp.block(0, 0, 3, 3).transpose() *
               vel_interp.block(0, 0, 3, 3));
  v_IinG = vel_interp.block(0, 3, 3, 1);

  // Finally get the interpolated velocities
  // NOTE: Rdot = R*skew(omega)
  // NOTE: Rdotdot = Rdot*skew(omega) + R*skew(alpha) =>
  // R^T*(Rdotdot-Rdot*skew(omega))=skew(alpha)
  Eigen::Matrix4d acc_interp =
      pose0 * (A0dotdot * A1 * A2 + A0 * A1dotdot * A2 + A0 * A1 * A2dotdot +
               2 * A0dot * A1dot * A2 + 2 * A0 * A1dot * A2dot +
               2 * A0dot * A1 * A2dot);
  Eigen::Matrix3d omegaskew =
      pose_interp.block(0, 0, 3, 3).transpose() * vel_interp.block(0, 0, 3, 3);
  alpha_IinI = Vee(pose_interp.block(0, 0, 3, 3).transpose() *
                   (acc_interp.block(0, 0, 3, 3) -
                    vel_interp.block(0, 0, 3, 3) * omegaskew));
  a_IinG = acc_interp.block(0, 3, 3, 1);
  return true;
}

bool BsplineSE3::find_bounding_poses(const double timestamp,
                                     const AlignedEigenMat4d& poses, double& t0,
                                     Eigen::Matrix4d& pose0, double& t1,
                                     Eigen::Matrix4d& pose1) {
  // Set the default values
  t0 = -1;
  t1 = -1;
  pose0 = Eigen::Matrix4d::Identity();
  pose1 = Eigen::Matrix4d::Identity();

  // Find the bounding poses
  bool found_older = false;
  bool found_newer = false;

  // Find the bounding poses for interpolation.
  auto lower_bound = poses.lower_bound(
      timestamp); // Finds timestamp or next(timestamp) if not available
  auto upper_bound = poses.upper_bound(timestamp); // Finds next(timestamp)

  if (lower_bound != poses.end()) {
    // Check that the lower bound is the timestamp.
    // If not then we move iterator to previous timestamp so that the timestamp
    // is bounded
    if (lower_bound->first == timestamp) {
      found_older = true;
    } else if (lower_bound != poses.begin()) {
      --lower_bound;
      found_older = true;
    }
  }

  if (upper_bound != poses.end()) { found_newer = true; }

  // If we found the older one, set it
  if (found_older) {
    t0 = lower_bound->first;
    pose0 = lower_bound->second;
  }

  // If we found the newest one, set it
  if (found_newer) {
    t1 = upper_bound->first;
    pose1 = upper_bound->second;
  }

  // Assert the timestamps
  if (found_older && found_newer) assert(t0 < t1);

  // Return true if we found both bounds
  return (found_older && found_newer);
}

bool BsplineSE3::get_2_boundary_points(const double timestamp,
                                       const AlignedEigenMat4d& poses,
                                       double& t0, Eigen::Matrix4d& pose0,
                                       double& t1, Eigen::Matrix4d& pose1) {
  // check if outside bounds
  if (timestamp < poses.begin()->first || timestamp > poses.rbegin()->first) {
    return false;
  }

  auto iter_mid = poses.begin();
  std::advance(iter_mid, poses.size() / 2);
  if (timestamp < iter_mid->first) {
    // get start boundary points
    auto r_iter = poses.begin();
    t0 = r_iter->first;
    pose0 = r_iter->second;
    r_iter++;
    t1 = r_iter->first;
    pose1 = r_iter->second;
  } else {
    // get end boundary points
    auto r_iter = poses.rbegin();
    t1 = r_iter->first;
    pose1 = r_iter->second;
    r_iter++;
    t0 = r_iter->first;
    pose0 = r_iter->second;
  }

  return true;
}

bool BsplineSE3::get_4_boundary_points(const double timestamp,
                                       const AlignedEigenMat4d& poses,
                                       double& t0, Eigen::Matrix4d& pose0,
                                       double& t1, Eigen::Matrix4d& pose1,
                                       double& t2, Eigen::Matrix4d& pose2,
                                       double& t3, Eigen::Matrix4d& pose3) {
  // check if outside bounds
  if (timestamp < poses.begin()->first || timestamp > poses.rbegin()->first) {
    return false;
  }

  auto iter_mid = poses.begin();
  std::advance(iter_mid, poses.size() / 2);
  if (timestamp < iter_mid->first) {
    // get start boundary points
    auto r_iter = poses.begin();
    t0 = r_iter->first;
    pose0 = r_iter->second;
    r_iter++;
    t1 = r_iter->first;
    pose1 = r_iter->second;
    r_iter++;
    t2 = r_iter->first;
    pose2 = r_iter->second;
    r_iter++;
    t3 = r_iter->first;
    pose3 = r_iter->second;
  } else {
    // get end boundary points
    auto r_iter = poses.rbegin();
    t3 = r_iter->first;
    pose3 = r_iter->second;
    r_iter++;
    t2 = r_iter->first;
    pose2 = r_iter->second;
    r_iter++;
    t1 = r_iter->first;
    pose1 = r_iter->second;
    r_iter++;
    t0 = r_iter->first;
    pose0 = r_iter->second;
  }

  return true;
}

bool BsplineSE3::find_bounding_control_points(
    const double timestamp, const AlignedEigenMat4d& poses, double& t0,
    Eigen::Matrix4d& pose0, double& t1, Eigen::Matrix4d& pose1, double& t2,
    Eigen::Matrix4d& pose2, double& t3, Eigen::Matrix4d& pose3) {
  // Set the default values
  t0 = -1;
  t1 = -1;
  t2 = -1;
  t3 = -1;
  pose0 = Eigen::Matrix4d::Identity();
  pose1 = Eigen::Matrix4d::Identity();
  pose2 = Eigen::Matrix4d::Identity();
  pose3 = Eigen::Matrix4d::Identity();

  // Get the two bounding poses
  bool success = find_bounding_poses(timestamp, poses, t1, pose1, t2, pose2);

  // if this fails, then grab boundary points
  if (!success) {
    return get_4_boundary_points(timestamp, poses, t0, pose0, t1, pose1, t2,
                                 pose2, t3, pose3);
  }

  // Now find the poses that are below and above
  auto iter_t1 = poses.find(t1);
  auto iter_t2 = poses.find(t2);

  // Check that t1 is not the first timestamp
  if (iter_t1 == poses.begin()) {
    return get_4_boundary_points(timestamp, poses, t0, pose0, t1, pose1, t2,
                                 pose2, t3, pose3);
  }

  // Move the older pose backwards in time
  // Move the newer one forwards in time
  auto iter_t0 = --iter_t1;
  auto iter_t3 = ++iter_t2;

  // Check that it is valid
  if (iter_t3 == poses.end()) {
    return get_4_boundary_points(timestamp, poses, t0, pose0, t1, pose1, t2,
                                 pose2, t3, pose3);
  }

  // Set the oldest one
  t0 = iter_t0->first;
  pose0 = iter_t0->second;

  // Set the newest one
  t3 = iter_t3->first;
  pose3 = iter_t3->second;

  // Assert the timestamps
  if (success) {
    assert(t0 < t1);
    assert(t1 < t2);
    assert(t2 < t3);
  }

  // Return true if we found all four bounding poses
  return success;
}

} // namespace beam
