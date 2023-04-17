#include "beam_mapping/Poses.h"
#include "beam_utils/bspline.h"
#include "beam_utils/se3.h"

#include <catch2/catch.hpp>
#include <iostream>

TEST_CASE("Spline Own Traj", "[spline.h]") {
  // generate example poses
  std::vector<beam::Pose> poses;
  beam::Pose p_first;
  p_first.timestampInNs = 0;
  p_first.T_FIXED_MOVING = Eigen::Matrix4d::Identity();

  poses.push_back(p_first);
  Eigen::VectorXd pert(6);
  pert << 0, -0.5, 1, 0.01, 0, 0;
  for (int i = 0; i < 50; i++) {
    beam::Pose p;
    p.timestampInNs = poses.back().timestampInNs + 1e9;
    p.T_FIXED_MOVING =
        beam::PerturbTransformDegM(poses.back().T_FIXED_MOVING, pert);
    poses.push_back(p);
  }

  // fit spline
  beam::BsplineSE3 spline;
  spline.feed_trajectory(poses);

  // save orig
  beam_mapping::Poses poses_to_save;
  for (const auto& p : poses) {
    ros::Time t;
    t.fromNSec(p.timestampInNs);
    poses_to_save.AddSingleTimeStamp(t);
    poses_to_save.AddSinglePose(p.T_FIXED_MOVING);
  }

  // save spline
  beam_mapping::Poses poses_to_save_spline;
  for (const auto& p : poses) {
    beam::Pose p_spline;
    p_spline.timestampInNs = p.timestampInNs;
    double stampS = static_cast<double>(p.timestampInNs * 1e-9);
    if (!spline.get_pose(stampS, p_spline.T_FIXED_MOVING)) { continue; }
    ros::Time t;
    t.fromNSec(p_spline.timestampInNs);
    REQUIRE(beam::ArePosesEqual(p_spline.T_FIXED_MOVING, p.T_FIXED_MOVING, 0.5,
                                0.005));
    poses_to_save_spline.AddSingleTimeStamp(t);
    poses_to_save_spline.AddSinglePose(p_spline.T_FIXED_MOVING);
  }

  /** Uncomment to view trajectories */
  // poses_to_save.WriteToFile("/home/nick/A_poses_orig.pcd", "PCD");
  // poses_to_save_spline.WriteToFile("/home/nick/A_poses_spline.pcd", "PCD");
  // poses_to_save.WriteToFile("/home/nick/poses_orig.json", "JSON");
  // poses_to_save_spline.WriteToFile("/home/nick/poses_spline.json", "JSON");
  /** */
}

TEST_CASE("Spline Extrapolate", "[spline.h]") {
  // generate example poses
  std::vector<beam::Pose> poses_gt;
  beam::Pose p_first;
  p_first.timestampInNs = 0;
  p_first.T_FIXED_MOVING = Eigen::Matrix4d::Identity();

  int traj_size = 120;
  poses_gt.push_back(p_first);
  Eigen::VectorXd pert(6);
  pert << 0, -0.5, 1, 0.02, 0.01, 0;
  for (int i = 0; i < traj_size; i++) {
    beam::Pose p;
    p.timestampInNs = poses_gt.back().timestampInNs + 1e9;
    p.T_FIXED_MOVING =
        beam::PerturbTransformDegM(poses_gt.back().T_FIXED_MOVING, pert);
    poses_gt.push_back(p);
  }

  // fit spline
  beam::BsplineSE3 spline;
  spline.feed_trajectory(poses_gt);

  // build trajectory up to middle of above poses
  std::vector<beam::Pose> cur_traj;
  for (int i = 0; i < traj_size / 2; i++) {
    cur_traj.push_back(poses_gt.at(i));
  }

  // go through the second half of the trajectory, build new spline for each
  // pose up to the time before, and predict next
  std::vector<beam::Pose> extrapolated_poses;
  for (int i = traj_size / 2 + 1; i < traj_size + 1; i++) {
    const auto& cur_gt_pose = poses_gt.at(i);

    // build new spline
    beam::BsplineSE3 cur_spline;
    cur_spline.feed_trajectory(cur_traj);

    // extrapolate
    beam::Pose p_extrapolated;
    p_extrapolated.timestampInNs = cur_gt_pose.timestampInNs;
    REQUIRE(cur_spline.extrapolate(cur_gt_pose.timestampInNs * 1e-9,
                                   p_extrapolated.T_FIXED_MOVING));
    extrapolated_poses.push_back(p_extrapolated);
    cur_traj.push_back(cur_gt_pose);

    REQUIRE(beam::ArePosesEqual(p_extrapolated.T_FIXED_MOVING,
                                cur_gt_pose.T_FIXED_MOVING, 0.1, 0.001));
  }

  // save (need to convert to beam_mapping::Poses)
  beam_mapping::Poses bm_poses_gt;
  beam_mapping::Poses bm_poses_spline;
  for (const auto& p : poses_gt) {
    beam::Pose p_spline;
    p_spline.timestampInNs = p.timestampInNs;
    double stampS = static_cast<double>(p.timestampInNs * 1e-9);
    if (!spline.get_pose(stampS, p_spline.T_FIXED_MOVING)) { continue; }
    ros::Time t;
    t.fromNSec(p_spline.timestampInNs);
    bm_poses_gt.AddSingleTimeStamp(t);
    bm_poses_gt.AddSinglePose(p.T_FIXED_MOVING);
    bm_poses_spline.AddSingleTimeStamp(t);
    bm_poses_spline.AddSinglePose(p_spline.T_FIXED_MOVING);
  }

  beam_mapping::Poses bm_poses_extr;
  for (const auto& p : extrapolated_poses) {
    ros::Time t;
    t.fromNSec(p.timestampInNs);
    bm_poses_extr.AddSingleTimeStamp(t);
    bm_poses_extr.AddSinglePose(p.T_FIXED_MOVING);
  }

  /** Uncomment to view trajectories */
  bm_poses_gt.WriteToFile("/home/nick/poses_gt.pcd", "PCD");
  bm_poses_spline.WriteToFile("/home/nick/poses_spline.pcd", "PCD");
  bm_poses_extr.WriteToFile("/home/nick/poses_extrapolated.pcd", "PCD");
}
