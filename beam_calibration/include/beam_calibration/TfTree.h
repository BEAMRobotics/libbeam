/** @file
 * @ingroup calibration
 * Includes all defects classes / functions
 *
 * @defgroup calibration
 * Calibration functions
 */

#pragma once

#include <beam_utils/math.h>
#include <geometry_msgs/TransformStamped.h>
#include <nlohmann/json.hpp>
#include <tf2/buffer_core.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <unordered_map>

namespace beam_calibration {
/** @addtogroup calibration
 *  @{ */

/**
 * @brief Class for managing extrinsic transformation tree using the tf2 library
 */
class TfTree {
public:
  /**
   * @brief Default constructor
   */
  TfTree() = default;

  /**
   * @brief Default constructor
   */
  ~TfTree() = default;

  /**
   * @brief Method for loading a tf tree from a .json file
   * @param file_location absolute path to json file
   */
  void LoadJSON(const std::string& file_location);

  /**
   * @brief Method for adding a transformation using an Affine3d
   * @param T new transform to add
   * @param to_frame
   * @param from_frame
   */
  void AddTransform(const Eigen::Affine3d& T, const std::string& to_frame,
                    const std::string& from_frame);

  /**
   * @brief Method for adding a transformation using an Affine3d
   * @param T new transform to add
   * @param to_frame
   * @param from_frame
   * @param time_stamp
   */
  void AddTransform(const Eigen::Affine3d& T, const std::string& to_frame,
                    const std::string& from_frame, const ros::Time& time_stamp);

  /**
   * @brief Method for adding transform via TransformStamped
   * @param T_ROS
   * @param is_static (default: false)
   */
  void AddTransform(const geometry_msgs::TransformStamped& T_ROS,
                    bool is_static = false);

  /**
   * @brief Method for retrieving a static transformation
   * @return Return the transformation requested as Affine3d object
   */
  Eigen::Affine3d GetTransformEigen(const std::string& to_frame,
                                    const std::string& from_frame) const;

  /**
   * @brief Method for retrieving a dynamic transformation
   * @return Return the transformation requested as Affine3d object
   * @param lookup_time
   */
  Eigen::Affine3d GetTransformEigen(const std::string& to_frame,
                                    const std::string& from_frame,
                                    const ros::Time& lookup_time) const;

  /**
   * @brief Method for looking up a dynamic transform
   * @param to_frame
   * @param from_frame
   * @param lookup_time
   * @return T_ROS
   */
  geometry_msgs::TransformStamped
      GetTransformROS(const std::string& to_frame,
                      const std::string& from_frame,
                      const ros::Time& lookup_time) const;

  /**
   * @brief Method for looking up a static transform
   * @param to_frame
   * @param from_frame
   * @return T_ROS
   */
  geometry_msgs::TransformStamped
      GetTransformROS(const std::string& to_frame,
                      const std::string& from_frame) const;

  /**
   * @brief Method for retrieving the date that the calibration was done
   * @return Return calibration date
   */
  std::string GetCalibrationDate() const;

  /**
   * @brief Method for setting the date that the calibration was done
   * @param Calibration date
   */
  void SetCalibrationDate(const std::string& calibration_date);

  ros::Time start_time{0};

  std::unordered_map<std::string, std::vector<std::string>>
      GetAllFrames() const {
    /**
     * @brief Method for getting all the frames in TfTree
     * @return Return unordered_map. First strings are from (parent) frames and
     * vectors of strings are to (child) frames
     */
    return frames_;
  }

  void Clear();
  /**
   * @brief Method for clearing all data in object
   */

private:
  /**
   * @brief Private method for setting a transform in the tf tree
   * @param T_ROS Transform being added from from_frame to to_frame
   * @param to_frame parent frame of a transform
   * @param from_frame child frame of a transform
   * @param time_stamp
   * @param is_static
   */
  void SetTransform(const geometry_msgs::TransformStamped& T_ROS,
                    const std::string& to_frame, const std::string& from_frame,
                    const ros::Time& time_stamp, bool is_static);

  /**
   * @brief Private method for looking up a transform
   * @param to_frame parent frame of a transform
   * @param from_frame child frame of a transform
   * @param time_stamp
   * @return T_ROS
   */
  geometry_msgs::TransformStamped
      LookupTransform(const std::string& to_frame,
                      const std::string& from_frame,
                      const ros::Time& time_stamp) const;

  /**
   * @brief Private method for converting an Eigen transform to a ROS geometry
   * message. It also does some checks
   * @param T Transform from from_frame to to_frame to be converted
   * @param to_frame parent frame of a transform
   * @param from_frame child frame of a transform
   * @param time_stamp
   * @return T_ROS
   */
  geometry_msgs::TransformStamped EigenToROS(const Eigen::Affine3d& T,
                                             const std::string& to_frame,
                                             const std::string& from_frame,
                                             const ros::Time& time_stamp) const;

  /**
   * @brief Private method for converting a ROS transform to Eigen transform
   * @param T_ROS Transform from from_frame to to_frame to be converted
   * @return T
   */
  Eigen::Affine3d
      ROSToEigen(const geometry_msgs::TransformStamped& T_ROS) const;

  /**
   * @brief Method for storing frame names in frames_ variable
   * @param to_frame child frame of a transform
   * @param from_frame parent frame of a transform
   */
  void InsertFrame(const std::string& to_frame, const std::string& from_frame);

  tf2::BufferCore Tree_{ros::Duration(1000)};
  std::string calibration_date_;
  bool is_calibration_date_set_ = false;

  // Stores all frame names. Key: Parent frame Value: Vector of child frames
  std::unordered_map<std::string, std::vector<std::string>> frames_;
};

/** @} group calibration */

} // namespace beam_calibration
