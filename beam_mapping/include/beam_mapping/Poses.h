/** @file
 * @ingroup mapping
 */

#pragma once

#include <Eigen/StdVector>
#include <Eigen/Dense>
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
   * @brief for adding bag_name to pose file
   * @param _bag_name
   */
  void AddBagName(const std::string& _bag_name);

  /**
   * @brief for getting bag_name
   * @return bag_name
   */
  std::string GetBagName();

  /**
   * @brief for adding pose_file_date to pose file
   * @param _pose_file_date
   */
  void AddPoseFileDate(const std::string& _pose_file_date);

  /**
   * @brief for getting pose_file_date
   * @return pose_file_date
   */
  std::string GetPoseFileDate();

  /**
   * @brief for adding fixed_frame to pose file
   * @param _fixed_frame
   */
  void AddFixedFrame(const std::string& _fixed_frame);

  /**
   * @brief for getting fixed_frame
   * @return fixed_frame
   */
  std::string GetFixedFrame();

  /**
   * @brief for adding moving_frame to pose file
   * @param _moving_frame
   */
  void AddMovingFrame(const std::string& _moving_frame);

  /**
   * @brief for getting moving_frame
   * @return moving_frame
   */
  std::string GetMovingFrame();

  /**
   * @brief for adding time stamps
   * @param _time_stamps
   */
  void AddTimeStamps(const std::vector<ros::Time>& _time_stamps);

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
   * @brief for adding poses
   * @param _poses
   */
  void AddPoses(
      const std::vector<Eigen::Affine3d,
                        Eigen::aligned_allocator<Eigen::Affine3d>>& _poses);

  /**
   * @brief for getting the poses
   * @return poses
   */
  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>
      GetPoses();

  /**
   * @brief for adding a single pose
   * @param pose
   */
  void AddSinglePose(const Eigen::Affine3d& pose);

  /**
   * @brief writes the pose file to the specified directory. The file will be
   * named: "poses_file_date"_poses.json
   * @param output_dir full path to directory at which to save pose file
   */
  void WriteToPoseFile(const std::string output_dir);

  /**
   * @brief loads the pose file
   * @param input_pose_file_path full path to pose file
   */
  void LoadPoseFile(const std::string input_pose_file_path);

  std::vector<ros::Time> time_stamps;
  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> poses;
  std::string bag_name, pose_file_date, fixed_frame, moving_frame;
};

/** @} group mapping */

} // namespace beam_mapping
