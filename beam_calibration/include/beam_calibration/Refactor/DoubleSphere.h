/** @file
 * @ingroup calibration
 */

#pragma once

#include "beam_calibration/include/Refactor/CameraModel.h"

namespace beam_calibration {

/**
 * @brief Derived class for pinhole camera model
 */
class DoubleSphere : public CameraModel {
public:
  /**
   * @brief Constructor. All code was implemented from the following paper:
   * https://arxiv.org/pdf/1807.08957.pdf
   * @param input_file path to input file
   */
  DoubleSphere(const std::string& file_path) override;

  /**
   * @brief Default destructor
   */
  ~DoubleSphere() override = default;

  /**
   * @brief Method for projecting a point into an image plane
   * @param point 3d point to be projected [x,y,z]^T
   * @param pixel reference to an optional vector with image coordinates after
   * point has been projected into the image plane [u,v]^T
   */
  void ProjectPoint(const Eigen::Vector3d& point,
                    std::optional<Eigen::Vector2i>& pixel) override;

  /**
   * @brief Method back projecting
   * @return Returns bearing vector
   * @param point [u,v]
   */
  void BackProject(const Eigen::Vector2i& pixel,
                   std::optional<Eigen::Vector3d>& ray) override;

protected:
  /**
   * @brief Method for validating the inputs. This will be called in the load
   * configuration file step and should validate the intrinsics (i.e. size) and
   * the type
   */
  void ValidateInputs() override;
  
  void LoadJSON(const std::string& file_path);

  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double eps_;
  double alpha_;
};

} // namespace beam_calibration
