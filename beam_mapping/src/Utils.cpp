#include <beam_mapping/Utils.h>

#include <tf2_eigen/tf2_eigen.h>

#include <beam_utils/log.h>

namespace beam_mapping { namespace utils {

void OdomMsgToPoses(const nav_msgs::Odometry& odom,
                    std::vector<Eigen::Matrix4d, beam::AlignMat4d>& poses,
                    std::vector<ros::Time>& timestamps,
                    std::string& fixed_frame, std::string& moving_frame) {
  if (fixed_frame.empty()) { fixed_frame = odom.header.frame_id; }
  if (moving_frame.empty()) { moving_frame = odom.child_frame_id; }
  timestamps.push_back(odom.header.stamp);
  Eigen::Affine3d T_FIXED_MOVING;
  Eigen::fromMsg(odom.pose.pose, T_FIXED_MOVING);
  poses.push_back(T_FIXED_MOVING.matrix());
}

void OdomMsgToPoses(const nav_msgs::Odometry& odom, pose_map_type& poses,
                    std::string& fixed_frame, std::string& moving_frame) {
  if (fixed_frame.empty()) { fixed_frame = odom.header.frame_id; }
  if (moving_frame.empty()) { moving_frame = odom.child_frame_id; }
  Eigen::Affine3d T_FIXED_MOVING;
  Eigen::fromMsg(odom.pose.pose, T_FIXED_MOVING);
  uint64_t stamp = odom.header.stamp.toNSec();

  // odometry does not have duplicates
  poses.emplace(stamp, T_FIXED_MOVING.matrix());
}

void PathMsgToPoses(const nav_msgs::Path& path,
                    std::vector<Eigen::Matrix4d, beam::AlignMat4d>& poses,
                    std::vector<ros::Time>& timestamps,
                    std::string& fixed_frame, std::string& moving_frame) {
  fixed_frame = path.header.frame_id;
  std::vector<geometry_msgs::PoseStamped> poses_stamped = path.poses;
  bool moving_frame_set = false;
  for (const auto& pose_stamped : poses_stamped) {
    if (!moving_frame_set && !pose_stamped.header.frame_id.empty()) {
      moving_frame = pose_stamped.header.frame_id;
      moving_frame_set = true;
    }

    timestamps.push_back(pose_stamped.header.stamp);
    Eigen::Affine3d T_MOVING_FIXED;
    Eigen::fromMsg(pose_stamped.pose, T_MOVING_FIXED);
    poses.push_back(T_MOVING_FIXED.matrix());
  }
}

int PathMsgToPoses(const nav_msgs::Path& path, pose_map_type& poses,
                   std::string& fixed_frame, std::string& moving_frame) {
  fixed_frame = path.header.frame_id;
  std::vector<geometry_msgs::PoseStamped> poses_stamped = path.poses;
  bool moving_frame_set = false;
  int duplicate_pose_stamps{0};
  for (const auto& pose_stamped : poses_stamped) {
    if (!moving_frame_set && !pose_stamped.header.frame_id.empty()) {
      moving_frame = pose_stamped.header.frame_id;
      moving_frame_set = true;
    }

    Eigen::Affine3d T_MOVING_FIXED;
    Eigen::fromMsg(pose_stamped.pose, T_MOVING_FIXED);
    uint64_t stamp = pose_stamped.header.stamp.toNSec();
    auto iter = poses.find(stamp);
    if (iter == poses.end()) {
      poses.emplace(stamp, T_MOVING_FIXED.matrix());
    } else {
      iter->second = T_MOVING_FIXED.matrix();
      duplicate_pose_stamps++;
    }
  }
  return duplicate_pose_stamps;
}

void PoseMapToTimeAndPoseVecs(
    const pose_map_type& pose_map,
    std::vector<Eigen::Matrix4d, beam::AlignMat4d>& poses,
    std::vector<ros::Time>& timestamps) {
  for (auto iter = pose_map.begin(); iter != pose_map.end(); iter++) {
    ros::Time stamp;
    stamp.fromNSec(iter->first);
    timestamps.push_back(stamp);
    poses.push_back(iter->second);
  }
}

}} // namespace beam_mapping::utils
