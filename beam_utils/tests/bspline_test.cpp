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
  pert << 0, 0, 10, 0.1, 0, 0;
  for (int i = 0; i < 30; i++) {
    beam::Pose p;
    p.timestampInNs = poses.back().timestampInNs + 1e9;
    p.T_FIXED_MOVING =
        beam::PerturbTransformDegM(poses.back().T_FIXED_MOVING, pert);
    poses.push_back(p);
  }

  // fit spline
  beam::BsplineSE3 spline;
  spline.feed_trajectory(poses);
  std::vector<beam::Pose> poses_spline;
  beam_mapping::Poses poses_to_save;
  beam_mapping::Poses poses_to_save_spline;
  for (const auto& p : poses) {
    beam::Pose p_spline;
    p_spline.timestampInNs = p.timestampInNs;
    double stampS = static_cast<double>(p.timestampInNs * 1e-9);
    if (!spline.get_pose(stampS, p_spline.T_FIXED_MOVING)) {
      std::cout << "COULD NOT GET POSE\n";
      continue;
    }
    poses_spline.push_back(p_spline);
    ros::Time t;
    t.fromNSec(p_spline.timestampInNs);
    REQUIRE(beam::ArePosesEqual(p_spline.T_FIXED_MOVING, p.T_FIXED_MOVING, 0.5,
                                0.005));
    poses_to_save.AddSingleTimeStamp(t);
    poses_to_save.AddSinglePose(p.T_FIXED_MOVING);
    poses_to_save_spline.AddSingleTimeStamp(t);
    poses_to_save_spline.AddSinglePose(p_spline.T_FIXED_MOVING);
  }

  /** Uncomment to view trajectories
  poses_to_save.WriteToFile("/home/nick/poses_orig.pcd", "PCD");
  poses_to_save_spline.WriteToFile("/home/nick/poses_spline.pcd", "PCD");
  poses_to_save.WriteToFile("/home/nick/poses_orig.json", "JSON");
  poses_to_save_spline.WriteToFile("/home/nick/poses_spline.json", "JSON");
  */
}
