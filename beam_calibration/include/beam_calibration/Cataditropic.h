/** @file
 * @ingroup calibration
 */

#pragma once

#include <beam_calibration/CameraModel.h>

namespace beam_calibration {

/**
 * @brief Derived class for Cataditropic model:
 * [https://www.robots.ox.ac.uk/~cmei/articles/single_viewpoint_calib_mei_07.pdf]
 */
class Cataditropic : public CameraModel {
public:
  /**
   * @brief Default constructor
   */
  Cataditropic() = default;

  /**
   * @brief Constructor.
   * @param input_file path to input file
   */
  Cataditropic(const std::string& file_path);

  /**
   * @brief Default destructor
   */
  ~Cataditropic() override = default;

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
  void Distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const;

  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double xi_;
  double k1_;
  double k2_;
  double p1_;
  double p2_;
  // inverse
  double m_inv_K11;
  double m_inv_K13;
  double m_inv_K22;
  double m_inv_K23;
};

/** @} group calibration */

} // namespace beam_calibration