/** @file
 * @ingroup mapping
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <ros/time.h>

namespace beam_mapping {
/** @addtogroup mapping
 *  @{ */

/**
 * @brief class for map builder
 */
class Poses {
public:
  /**
   * @brief default constructor
   */
  Poses() = default;

  /**
   * @brief Default destructor
   */
  ~Poses() = default;

  /**
   * @brief clears all content
   */
  void Clear();

  /**
   * @brief for setting bag_name to pose file
   * @param _bag_name
   */
  void SetBagName(const std::string& _bag_name);

  /**
   * @brief for getting bag_name
   * @return bag_name
   */
  std::string GetBagName();

  /**
   * @brief for setting pose_file_date to pose file
   * @param _pose_file_date
   */
  void SetPoseFileDate(const std::string& _pose_file_date);

  /**
   * @brief for getting pose_file_date
   * @return pose_file_date
   */
  std::string GetPoseFileDate();

  /**
   * @brief for setting fixed_frame to pose file
   * @param _fixed_frame
   */
  void SetFixedFrame(const std::string& _fixed_frame);

  /**
   * @brief for getting fixed_frame
   * @return fixed_frame
   */
  std::string GetFixedFrame();

  /**
   * @brief for setting moving_frame to pose file
   * @param _moving_frame
   */
  void SetMovingFrame(const std::string& _moving_frame);

  /**
   * @brief for getting moving_frame
   * @return moving_frame
   */
  std::string GetMovingFrame();

  /**
   * @brief for setting time stamps
   * @param _time_stamps
   */
  void SetTimeStamps(const std::vector<ros::Time>& _time_stamps);

  /**
   * @brief for getting the time stamps
   * @return time_stamps
   */
  std::vector<ros::Time> GetTimeStamps();

  /**
   * @brief for adding a single time stamp
   * @param time_stamp
   */
  void AddSingleTimeStamp(const ros::Time& time_stamp);

  /**
   * @brief for setting poses
   * @param _poses transforms from fixed frame to moving frame
   */
  void SetPoses(
      const std::vector<Eigen::Affine3d,
                        Eigen::aligned_allocator<Eigen::Affine3d>>& _poses);

  /**
   * @brief for getting the poses
   * @return poses transforms from fixed frame to moving frame
   */
  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>
      GetPoses();

  /**
   * @brief for adding a single pose
   * @param pose transform from fixed frame to moving frame
   */
  void AddSinglePose(const Eigen::Affine3d& pose);

  /**
   * @brief writes the pose file to the specified directory as JSON type. If a
   * directory is given (i.e. ending in /) the file will be named:
   * "poses_file_date"_poses.json. If a full filename is given (i.e.
   * /path/filename.json) it will keep that name.
   * @param output_dir full path to directory at which to save pose file
   */
  void WriteToJSON(const std::string output_dir);

  /**
   * @brief loads the pose file in JSON format
   * @param input_pose_file_path full path to pose file
   */
  void LoadFromJSON(const std::string input_pose_file_path);

  /**
   * @brief writes the pose file to the specified directory as TXT type. If a
   * directory is given (i.e. ending in /) the file will be named:
   * "poses_file_date"_poses.txt. If a full filename is given (i.e.
   * /path/filename.txt) it will keep that name.
   * @param output_dir full path to directory at which to save pose file
   */
  void WriteToTXT(const std::string output_dir);

  /**
   * @brief loads the pose file in txt format
   * @param input_pose_file_path full path to pose file
   */
  void LoadFromTXT(const std::string input_pose_file_path);

  /**
   * @brief writes the pose file to the specified directory as PLY type. If a
   * directory is given (i.e. ending in /) the file will be named:
   * "poses_file_date"_poses.ply. If a full filename is given (i.e.
   * /path/filename.ply) it will keep that name.
   * @param output_dir full path to directory at which to save pose file
   */
  void WriteToPLY(const std::string output_dir);

  /**
   * @brief loads the pose file in PLY format
   * @param input_pose_file_path full path to pose file
   */
  void LoadFromPLY(const std::string input_pose_file_path);

  /**
   * @brief loads the pose object using the odometry from a bag file
   * @param bag_file_path full path to bag file
   * @param odom_topic
   */
  void LoadFromBAG(const std::string bag_file_path,
                   const std::string odom_topic);

  std::vector<ros::Time> time_stamps;
  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> poses;
  std::string bag_name, pose_file_date, fixed_frame, moving_frame;
};

/** @} group mapping */

} // namespace beam_mapping
