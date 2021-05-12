/** @file
 * @ingroup calibration
 */

#pragma once
#include <beam_calibration/CameraModel.h>

#include <ladybug/ladybug.h>
#include <ladybug/ladybuggeom.h>
#include <ladybug/ladybugrenderer.h>

namespace beam_calibration {

/**
 * @brief Derived class for camera model
 * Due to being a specific case ladybug cannot be instantiated in the
 * Create or LoadJSON methods and must be manually created using
 * the conf file from the ladybug SDK
 */
class Ladybug : public CameraModel {
public:
  /**
   * @brief Default constructor
   */
  Ladybug() = default;

  /**
   * @brief Constructor
   * @param input_file path to input file
   */
  Ladybug(const std::string& file_path);

  /**
   * @brief Default destructor
   */
  ~Ladybug() override = default;

  /**
   * @brief Method to perform a deep copying of this object
   */
  std::shared_ptr<CameraModel> Clone() override;

  /**
   * @brief Method for projecting a point into an image plane (continous)
   * @param point 3d point to be projected [x,y,z]^T
   * @param outside_domain optional parameter, set if point is outside camera
   * model domain
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

  /**
   * @brief Method for setting which camera to use
   * @param id of the camera to use
   */
  void SetCameraID(unsigned int id) override;

protected:
  /**
   * @brief Checks if recent api call caused an error
   */
  void LadybugCheckError();

  LadybugContext lb_context_;
  LadybugError lb_error_;
  const unsigned int LB_FULL_WIDTH_ = 2048;
  const unsigned int LB_FULL_HEIGHT_ = 2464;

  // static bool outside_domain_default_;

  double focal_length_;
  double cx_;
  double cy_;
};

} // namespace beam_calibration
