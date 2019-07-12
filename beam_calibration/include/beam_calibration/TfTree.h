/** @file
 * @ingroup calibration
 * Includes all defects classes / functions
 *
 * @defgroup calibration
 * Calibration functions
 */

#pragma once

#include <beam_utils/math.hpp>
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
  void LoadJSON(std::string& file_location);

  /**
   * @brief Method for adding a transformation using an Affine3d
   * @param Tnew new transform to add
   * @param to_frame
   * @param from_frame
   */
  void AddTransform(Eigen::Affine3d& Tnew, std::string& to_frame,
                    std::string& from_frame);

  /**
   * @brief Method for adding a transformation using an Affine3d
   * @param Tnew new transform to add
   * @param to_frame
   * @param from_frame
   * @param time_stamp
   */
  void AddTransform(Eigen::Affine3d& Tnew, std::string& to_frame,
                    std::string& from_frame, ros::Time time_stamp);

  /**
   * @brief Method for adding transform via TransformStamped
   * @param msg
   */
  void AddTransform(geometry_msgs::TransformStamped msg,
                    bool is_static = false);

  /**
   * @brief Method for retrieving a static transformation
   * @return Return the transformation requested as Affine3d object
   */
  Eigen::Affine3d GetTransformEigen(std::string& to_frame,
                                    std::string& from_frame);

  /**
   * @brief Method for retrieving a dynamic transformation
   * @return Return the transformation requested as Affine3d object
   * @param lookup_time
   */
  Eigen::Affine3d GetTransformEigen(std::string& to_frame,
                                    std::string& from_frame,
                                    ros::Time& lookup_time);

  /**
   * @brief Method for looking up dynamic transform
   * @param to_frame
   * @param from_frame
   * @param lookup_time
   * @return Transform Stamped
   */
  geometry_msgs::TransformStamped GetTransformROS(std::string& to_frame,
                                                  std::string& from_frame,
                                                  ros::Time& lookup_time);

  /**
   * @brief Method for retrieving the date that the calibration was done
   * @return Return calibration date
   */
  std::string GetCalibrationDate();

  /**
   * @brief Method for setting the date that the calibration was done
   * @param Calibration date
   */
  void SetCalibrationDate(std::string& calibration_date);

  ros::Time start_time{0};

  /**
   * @brief Method for getting all the frames in TfTree
   * @return Return unordered_map. First strings are from (parent) frames and
   * vectors of strings are to (child) frames
   */
  std::unordered_map<std::string, std::vector<std::string>> GetAllFrames() {
    return frames_;
  }

private:
  /**
   * @brief Private method for setting a transform in the tf tree
   * @param Tnew Transform being added from from_frame to to_frame
   * @param to_frame child frame of a transform
   * @param from_frame parent frame of a transform
   */
  void SetTransform(Eigen::Affine3d& Tnew, std::string& to_frame,
                    std::string& from_frame);

  /**
   * @brief Method for storing frame names in frames_ variable
   * @param to_frame child frame of a transform
   * @param from_frame parent frame of a transform
   */
  void InsertFrame(std::string& to_frame, std::string& from_frame);

  tf2::BufferCore Tree_{ros::Duration(1000)};
  std::string calibration_date_;
  bool is_calibration_date_set_ = false;

  // Stores all frame names. Key: Parent frame Value: Vector of child frames
  std::unordered_map<std::string, std::vector<std::string>> frames_;
};

/** @} group calibration */

} // namespace beam_calibration
