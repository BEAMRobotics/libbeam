/** @file
 * @ingroup calibration
 * Includes all defects classes / functions
 *
 * @defgroup calibration
 * Calibration functions
 */

#pragma once

#include <tf2/buffer_core.h>
#include <beam_utils/log.hpp>
#include <beam_utils/math.hpp>
#include <fstream>
#include <iostream>
#include <geometry_msgs/TransformStamped.h>
#include <nlohmann/json.hpp>
#include <tf2_eigen/tf2_eigen.h>

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
   */
  void AddTransform(Eigen::Affine3d& Tnew, std::string& to_frame,
                    std::string& from_frame);

  /**
   * @brief Method for retrieving a transformation
   * @return Return the transformation requested as Affine3d object
   */
  Eigen::Affine3d GetTransform(std::string& to_frame, std::string& from_frame);

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

private:

  void SetTransform(Eigen::Affine3d& Tnew, std::string& to_frame,
                    std::string& from_frame);

  tf2::BufferCore Tree_;
  std::string calibration_date_;
  bool is_calibration_date_set_ = false;
};

/** @} group calibration */

} // namespace beam_calibration
