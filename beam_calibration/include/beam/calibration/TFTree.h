#pragma once
#include <string>
#include "beam/utils/math.hpp"
#include <tf2/buffer_core.h>


namespace beam_calibration {

class TFTree {
public:
  /**
   * @brief Default constructor
   */
  TFTree() = default;

  /**
   * @brief Default constructor
   */
  ~TFTree() = default;

  /**
   * @brief Method for adding a transformation using a 4x4 matrix
   */
  void AddTransform(beam::Mat4 Tnew, std::string from_frame,
                    std::string to_frame, std::string calib_date);

  /**
   * @brief Method for adding a transformation using an Affine3d
   */
  void AddTransform(Eigen::Affine3d Tnew, std::string from_frame,
                    std::string to_frame, std::string calib_date);

  /**
   * @brief Method for retrieving a transformation
   * @return Return the transformation requested as Affine3d object
   */
  Eigen::Affine3d GetTransform(std::string from_frame, std::string to_frame);

  /**
   * @brief Method for retrieving a transformation
   * @return Return the most recent calibration date from that portion of the
   * tree
   */
  std::string GetTransformDate(std::string from_frame, std::string to_frame);

private:
  // tf2::TransformStorage Tree;
  tf2::BufferCore Tree_;
};

} // namespace beam_calibration
