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
   * @param[in] in_point 3d point to be projected [x,y,z]^T
   * @param[out] out_pixel pixel the point projects to
   * @param[in] J optional param to compute the jacobian
   * @param[out] in_image_plane true if the pixel is outside of the image plane
   * @return whether the input point is in the domain of the function
   */
  bool ProjectPoint(const Eigen::Vector3d& in_point, Eigen::Vector2d& out_pixel,
                    bool& in_image_plane,
                    std::shared_ptr<Eigen::MatrixXd> J = nullptr) override;

  /**
   * @brief Method back projecting
   * @param[in] in_pixel pixel to back project
   * @param[out] out_point ray towards the input pixel
   * @return return whether the input pixel is in the domain of the function
   */
  bool BackProject(const Eigen::Vector2i& in_pixel,
                   Eigen::Vector3d& out_point) override;

  /**
   * @brief Method for setting which camera to use
   * @param id of the camera to use
   */
  void SetCameraID(unsigned int id) override;

  /**
   * @brief Method for checking if a 3d point is projectable
   * @return Returns boolean
   * @param point
   */
  bool InProjectionDomain(const Eigen::Vector3d& point) override;

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
