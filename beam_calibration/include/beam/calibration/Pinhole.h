#pragma once
#include "beam/calibration/Intrinsics.h"
#include "beam/utils/math.hpp"

namespace beam_calibration {

/**
 * @brief Derived class for pinhole intrinsics
 */
class Pinhole : public Intrinsics {
public:
  /**
   * @brief Default constructor
   */
  Pinhole() = default;

  /**
   * @brief constructor
   */
  Pinhole(double fx, double fy, double cx, double cy);

  /**
   * @brief constructor
   */
  Pinhole(beam::Mat3 K);

  /**
   * @brief Default constructor
   */
  ~Pinhole() override = default;

  /**
   * @brief Get the type of intrinsic
   * @return Returns type as one of intrinsics specified in the enum
   * IntrinsicsType
   */
  IntrinsicsType GetType() const override { return IntrinsicsType::PINHOLE; };

  /**
   * @brief Method for adding the frame id
   * @param frame_id frame associated with the intrinsics calibration object
   */
  void AddFrameId(std::string frame_id) override;

  /**
   * @brief Method for returning the frame id of an intrinsics
   * calibration object
   * @return Returns frame id
   */
  std::string GetFrameId() override;

  /**
   * @brief Method for adding the image dimensions
   * @param img_dims dimensions of the images taken by the camera associated
   * with this intrinsics object: [height, width]^T
   */
  void AddImgDims(beam::Vec2 img_dims) override;

  /**
   * @brief Method for getting the image dimensions
   * @return img_dims dimensions of the images taken by the camera associated
   * with this intrinsics object: [height, width]^T
   */
  beam::Vec2 GetImgDims() override;

  /**
   * @brief Method for returning K matrix
   * @return intrinsics matrix K_
   */
  beam::Mat3 GetK();

  /**
   * @brief Method for returning y focal length
   * @return Fy
   */
  double GetFy();

  /**
   * @brief Method for returning x focal length
   * @return Fx
   */
  double GetFx();

  /**
   * @brief Method for returning optical center in x direction
   * @return Cx
   */
  double GetCx();

  /**
   * @brief Method for returning optical center in y direction
   * @return Cy
   */
  double GetCy();

  /**
   * @brief Method for checking if the intinsic parameters have been set
   * @return Returns true if K and distortion parameters have been assigned
   */
  bool IsFull();

  /**
   * @brief Method for adding tangential distortion parameters
   */
  void AddTanDist(beam::Vec2 tan_coeffs);

  /**
   * @brief Method for adding radial distortion parameters
   */
  void AddRadDist(beam::VecX rad_coeffs);

  /**
   * @brief Method for getting tangential distortion parameters
   */
  beam::Vec2 GetTanDist();

  /**
   * @brief Method for getting radial distortion parameters
   */
  beam::VecX GetRadDist();

  /**
   * @brief Method for projecting a point into an image plane
   * @return Returns image coordinates after point has been projected into image
   * plane.
   */
  beam::Vec2 ProjectPoint(beam::Vec3 X);

  /**
   * @brief Method for projecting a point in homographic form into an image
   * plane
   * @return Returns image coordinates after point has been projected into image
   * plane.
   */
  beam::Vec2 ProjectPoint(beam::Vec4 X);

  /**
   * @brief Method for projecting a point into an image plane where the image is
   * distorted.
   * See:
   * https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
   * @return Returns image coordinates after point has been projected into image
   * plane.
   */
  beam::Vec2 ProjectDistortedPoint(beam::Vec3 X);

  /**
   * @brief Method for projecting a point in homographic form into an image
   * plane where the image is distorted.
   * See:
   * https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
   * @return Returns image coordinates after point has been projected into image
   * plane.
   */
  beam::Vec2 ProjectDistortedPoint(beam::Vec4 X);

private:
  /**
   * @brief This applies the projection for to images that are not distorted
   * @return Returns image coordinates after point has been projected into image
   * plane.
   */
  beam::Vec2 ApplyProjection(beam::Vec3 X);

  /**
   * @brief This applies the projection for to images that are not distorted
   * @return Returns image coordinates after point has been projected into image
   * plane.
   */
  beam::Vec2 ApplyDistortedProjection(beam::Vec3 X);

  // Variables for storing intrinsic information
  std::string frame_id_;
  beam::Vec2 img_dims_;
  beam::Mat3 K_;
  beam::Vec2 tan_coeffs_;
  beam::VecX rad_coeffs_;
  bool is_full_ = false, is_rad_distortion_valid_ = false,
       is_tan_distortion_valid_ = false;
};

} // namespace beam_calibration
