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
