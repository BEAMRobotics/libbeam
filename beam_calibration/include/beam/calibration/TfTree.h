/** @file
 * @ingroup calibration
 * Includes all defects classes / functions
 *
 * @defgroup calibration
 * Calibration functions
 */

#pragma once
#include "beam/utils/math.hpp"
#include <string>
#include <tf2/buffer_core.h>

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
   * @brief Method for adding a transformation using an Affine3d
   */
  void AddTransform(Eigen::Affine3d& Tnew, std::string& to_frame,
                    std::string& from_frame, std::string& calib_date);

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

private:
  /**
   * @brief Method for setting the date that the calibration was done
   * @param Calibration date
   */
  void SetCalibrationDate(std::string& calibraiton_date);

  tf2::BufferCore Tree_;
  std::string calibraiton_date_;
  bool is_calibration_date_set_ = false;
};

/** @} group calibration */

} // namespace beam_calibration
