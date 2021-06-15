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
   * @brief Method for checking if a 3d point is projectable
   * @return Returns boolean
   * @param point
   */
  bool InProjectionDomain(const Eigen::Vector3d& point) override;

protected:
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double eps_;
  double alpha_;
};

} // namespace beam_calibration
