/** @file
 * @ingroup calibration
 */

#pragma once

#include <beam_calibration/CameraModel.h>

namespace beam_calibration {

/**
 * @brief Derived class for pinhole camera model
 */
class Radtan : public CameraModel {
public:
  /**
   * @brief Default constructor
   */
  Radtan() = default;

  /**
   * @brief Constructor
   * @param input_file path to input file
   */
  Radtan(const std::string& file_path);

  /**
   * @brief Constructor specific to Radtan which creates a model without needing
   * a file
   * @param image_height
   * @param image_width
   * @param intrinsics vector of doubles of dimension 8 x 1
   */
  Radtan(uint32_t image_height, uint32_t image_width,
         const Eigen::Matrix<double, 8, 1>& intrinsics);

  /**
   * @brief Default destructor
   */
  ~Radtan() override = default;

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
   * @brief Method for undistorting an image based on camera's distortion
   * @param image_input image to be undistorted
   * @param image_output reference to Mat obejct to store output
   */
  void UndistortImage(const cv::Mat& image_input, cv::Mat& image_output);

  /**
   * @brief Method for checking if a 3d point is projectable
   * @return Returns boolean
   * @param point
   */
  bool InProjectionDomain(const Eigen::Vector3d& point) override;

protected:
  /**
   * @brief Method to distort point
   * @return Vec2 distorted point
   * @param pixel to distort
   */
  Eigen::Vector2d DistortPixel(const Eigen::Vector2d& pixel);

  /**
   * @brief Method to compute jacobian of the distortion function
   * @return Jacobian
   * @param pixel to compute jacobian around
   */
  Eigen::Matrix2d ComputeDistortionJacobian(const Eigen::Vector2d& pixel);

  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double k1_;
  double k2_;
  double p1_;
  double p2_;
};

/** @} group calibration */

} // namespace beam_calibration
