/** @file
 * @ingroup mapping
 */

#pragma once

#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <beam_utils/math.h>
#include <beam_utils/pointclouds.h>

namespace beam_mapping {
/** @addtogroup mapping
 *  @{ */

using pose_map_type = std::map<
    uint64_t, Eigen::Matrix4d, std::less<uint64_t>,
    Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Matrix4d> > >;

namespace utils {

/**
 * @brief convert a ROS odometry message to a vector of transforms and a vector
 * of timestamps
 * @param path input odometry msg
 * @param poses reference to output poses. NOTE: This will not override the
 * vector, it will add to it (push back).
 * @param timestamps reference to output timestamps. NOTE: This will not
 * override the vector, it will add to it (push back).
 * @param fixed_frame reference to fixed frame to fill in. This will come from
 * the odometry message header
 * @param moving_frame reference to the moving frame to fill in. This will come
 * from the odometry message header
 */
void OdomMsgToPoses(const nav_msgs::Odometry& odom,
                    std::vector<Eigen::Matrix4d, beam::AlignMat4d>& poses,
                    std::vector<ros::Time>& timestamps,
                    std::string& fixed_frame, std::string& moving_frame);

/**
 * @brief convert a ROS odometry message to a vector of transforms and a vector
 * of timestamps
 * @param path input odometry msg
 * @param poses reference to pose map. NOTE: this will not override poses,
 * but add to them
 * @param fixed_frame reference to fixed frame to fill in. This will come from
 * the odometry message header
 * @param moving_frame reference to the moving frame to fill in. This will come
 * from the odometry message header
 */
void OdomMsgToPoses(const nav_msgs::Odometry& odom, pose_map_type& poses,
                    std::string& fixed_frame, std::string& moving_frame);

/**
 * @brief convert a ROS path message to a vector of transforms and a vector of
 * timestamps
 * @param path input path msg
 * @param poses reference to output poses. NOTE: This will not override the
 * vector, it will add to it (push back).
 * @param timestamps reference to output timestamps. NOTE: This will not
 * override the vector, it will add to it (push back).
 * @param fixed_frame reference to fixed frame to fill in. This will come from
 * the path message header
 * @param moving_frame reference to the moving frame to fill in. This will come
 * from the first non-empty header frame id from the pose messages
 */
void PathMsgToPoses(const nav_msgs::Path& path,
                    std::vector<Eigen::Matrix4d, beam::AlignMat4d>& poses,
                    std::vector<ros::Time>& timestamps,
                    std::string& fixed_frame, std::string& moving_frame);

/**
 * @brief convert a ROS path message to an ordered map of pose_map_type. If
 * timestamps are duplicated, newer poses will override old ones.
 * @param path input path msg
 * @param pose_map reference to pose map. NOTE: this will not override poses,
 * but add to them
 * @param fixed_frame reference to fixed frame to fill in. This will come from
 * the path message header
 * @param moving_frame reference to the moving frame to fill in. This will come
 * from the first non-empty header frame id from the pose messages
 * @return number of duplicate poses
 */
int PathMsgToPoses(const nav_msgs::Path& path, pose_map_type& poses,
                   std::string& fixed_frame, std::string& moving_frame);

/**
 * @brief convert pose_map_type (see above definition) to a vector of poses and
 * a vector of ros timestamps. NOTE: this adds to the vectors in order of
 * increasing timestamps, and does not override
 * @param pose_map uint64_t (time) -> Matrix4d
 * @param poses reference to poses vector
 * @param timestamps reference to timestamps vector
 */
void PoseMapToTimeAndPoseVecs(
    const pose_map_type& pose_map,
    std::vector<Eigen::Matrix4d, beam::AlignMat4d>& poses,
    std::vector<ros::Time>& timestamps);

} // namespace utils

/** @} group mapping */

} // namespace beam_mapping
