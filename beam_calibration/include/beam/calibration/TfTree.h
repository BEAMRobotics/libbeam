#pragma once
#include "beam/utils/math.hpp"
#include <string>
#include <tf2/buffer_core.h>

namespace beam_calibration {

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
  void AddTransform(Eigen::Affine3d &Tnew, std::string &from_frame,
                    std::string &to_frame, std::string &calib_date);

  /**
   * @brief Method for retrieving a transformation
   * @return Return the transformation requested as Affine3d object
   */
  Eigen::Affine3d GetTransform(std::string &from_frame, std::string &to_frame);

  /**
   * @brief Method for setting the date that the calibration was done
   * @param Calibration date
   */
  void SetCalibrationDate(std::string &calibraiton_date);

  /**
   * @brief Method for retrieving the date that the calibration was done
   * @return Return calibration date
   */
  std::string GetCalibrationDate();

private:
  // tf2::TransformStorage Tree;
  tf2::BufferCore Tree_;
  std::string calibraiton_date_;
  bool is_calibration_date_set_ = false;
};

} // namespace beam_calibration
