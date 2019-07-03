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
   * @param bag_name
   */
  void AddBagName(const std::string& bag_name);

  /**
   * @brief for getting bag_name
   * @return bag_name
   */
  std::string GetBagName();

  /**
   * @brief for adding pose_file_date to pose file
   * @param pose_file_date
   */
  void AddPoseFileDate(const std::string& pose_file_date);

  /**
   * @brief for getting pose_file_date
   * @return pose_file_date
   */
  std::string GetPoseFileDate();

  /**
   * @brief for adding fixed_frame to pose file
   * @param fixed_frame
   */
  void AddFixedFrame(const std::string& fixed_frame);

  /**
   * @brief for getting fixed_frame
   * @return fixed_frame
   */
  std::string GetFixedFrame();

  /**
   * @brief for adding moving_frame to pose file
   * @param moving_frame
   */
  void AddMovingFrame(const std::string& moving_frame);

  /**
   * @brief for getting moving_frame
   * @return moving_frame
   */
  std::string GetMovingFrame();

  /**
   * @brief for adding time stamps
   * @param time_stamps
   */
  void AddTimeStamps(const std::vector<ros::Time>& time_stamps);

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
   * @param poses
   */
  void AddPoses(
      const std::vector<Eigen::Affine3d,
                        Eigen::aligned_allocator<Eigen::Affine3d>>& poses);

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

private:
  std::string bag_name_, pose_file_date_, fixed_frame_, moving_frame_;
  std::vector<ros::Time> time_stamps_;
  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>
      poses_;
};

/** @} group mapping */

} // namespace beam_mapping
