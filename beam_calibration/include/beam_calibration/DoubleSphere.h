/** @file
 * @ingroup calibration
 */

#pragma once

#include <beam_calibration/CameraModel.h>

namespace beam_calibration {

/**
 * @brief Derived class for Double Sphere model
 */
class DoubleSphere : public CameraModel {
public:
  /**
   * @brief Default constructor
   */
  DoubleSphere() = default;

  /**
   * @brief Constructor. All code was implemented from the following paper:
   * https://arxiv.org/pdf/1807.08957.pdf
   * @param input_file path to input file
   */
  DoubleSphere(const std::string& file_path);

  /**
   * @brief Default destructor
   */
  ~DoubleSphere() override = default;

  /**
   * @brief Method to perform a deep copying of this object
   */
  std::shared_ptr<CameraModel> Clone() override;

  /**
   * @brief Method for projecting a point into an image plane (continous)
   * @param point 3d point to be projected [x,y,z]^T
   * @param outside_domain optional parameter, set if point is outside camera
   * model domain point has been projected into the image plane [u,v]^T
   */
  beam::opt<Eigen::Vector2d> ProjectPointPrecise(
      const Eigen::Vector3d& point,
      bool& outside_domain = outside_domain_default_) override;

  /**
   * @brief Method for projecting a point into an image plane
   * @return projected point
   * @param point 3d point to be projected [x,y,z]^T
   * @param outside_domain optional parameter, set if point is outside camera
   * model domain
   */
  beam::opt<Eigen::Vector2i>
      ProjectPoint(const Eigen::Vector3d& point,
                   bool& outside_domain = outside_domain_default_) override;

  /**
   * @brief Overload projection function for computing jacobian of projection
   * @return projected point
   * @param point 3d point to be projected [x,y,z]^T
   * @param J 2 x 3 projection jacobian.
   * @param outside_domain optional parameter, set if point is outside camera
   * model domain For ProjectPoint: [u,v]^T = [P1(x, y, z), P2(x, y, z)]^T J = |
   * dP1/dx , dP1/dy, dP1/dz | | dP2/dx , dP2/dy, dP2/dz |
   */
  beam::opt<Eigen::Vector2i>
      ProjectPoint(const Eigen::Vector3d& point, Eigen::MatrixXd& J,
                   bool& outside_domain = outside_domain_default_) override;

  /**
   * @brief Method back projecting
   * @return Returns bearing vector
   * @param point [u,v]
   */
  beam::opt<Eigen::Vector3d> BackProject(const Eigen::Vector2i& pixel) override;

protected:
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double eps_;
  double alpha_;
};

} // namespace beam_calibration
