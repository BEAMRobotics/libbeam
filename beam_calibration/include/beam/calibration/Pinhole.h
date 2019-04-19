/** @file
 * @ingroup calibration
 */

#pragma once
#include "beam/calibration/Intrinsics.h"
#include <beam/utils/log.hpp>
#include <beam/utils/math.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

namespace beam_calibration {
/** @addtogroup calibration
 *  @{ */

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
   * @param fx focal length in x diretion
   * @param fy focal length in y diretion
   * @param cx optical center in x diretion
   * @param cy optical center in y diretion
   */
  Pinhole(double &fx, double &fy, double &cx, double &cy);

  /**
   * @brief constructor
   * @param K intrinsics matrix:
   * K = fx  0 cx
   *     0  fy cy
   *     0   0  1
   */
  Pinhole(beam::Mat3 &K);

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
   * @brief Method for loading a pinhole calibration from a .json file
   * @param file_location absolute path to json file
   */
  void LoadJSON(std::string &file_location);

  /**
   * @brief Method for adding the frame id
   * @param frame_id frame associated with the intrinsics calibration object
   */
  void SetFrameId(std::string &frame_id) override;

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
  void SetImgDims(beam::Vec2 &img_dims) override;

  /**
   * @brief Method for getting the image dimensions
   * @return img_dims dimensions of the images taken by the camera associated
   * with this intrinsics object: [height, width]^T
   */
  beam::Vec2 GetImgDims() override;

  /**
   * @brief Method for adding K matrix
   * @param K intrinsics matrix
   */
  void SetK(beam::Mat3 K) override;

  /**
   * @brief Method for returning K matrix
   * @return intrinsics matrix K_
   */
  beam::Mat3 GetK() override;

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
   * @brief Method for retrieving the date that the calibration was done
   * @return Return calibration date
   */
  std::string GetCalibrationDate();

  /**
   * @brief Method for setting the date that the calibration was done
   * @param Calibration date
   */
  void SetCalibrationDate(std::string& calibration_date);

  /**
   * @brief Method for checking if the intinsic parameters have been set
   * @return Returns true if K and distortion parameters have been assigned
   */
  bool IsKFull();

  /**
   * @brief Method for adding tangential distortion parameters
   * @param tan_coeffs vector of length two with the tangential distortion
   * coefficients
   */
  void SetTanDist(beam::Vec2 &tan_coeffs);

  /**
   * @brief Method for adding radial distortion parameters
   * @param rad_coeffs vector of length 3 to 6 with the radial distortion
   * coefficients
   */
  void SetRadDist(beam::VecX rad_coeffs);

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
   * @param X point to be projected. Not in homographic form
   */
  beam::Vec2 ProjectPoint(beam::Vec3 &X);

  /**
   * @brief Method for projecting a point in homographic form into an image
   * plane
   * @return Returns image coordinates after point has been projected into image
   * plane.
   * @param X point to be projected. In homographic form
   */
  beam::Vec2 ProjectPoint(beam::Vec4 &X);

  /**
   * @brief Method for projecting a point into an image plane where the image is
   * distorted.
   * See:
   * https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
   * @return Returns image coordinates after point has been projected into image
   * plane.
   * @param X point to be projected. Not in homographic form
   */
  beam::Vec2 ProjectDistortedPoint(beam::Vec3 &X);

  /**
   * @brief Method for projecting a point in homographic form into an image
   * plane where the image is distorted.
   * See:
   * https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
   * @return Returns image coordinates after point has been projected into image
   * plane.
   * @param X point to be projected. In homographic form
   */
  beam::Vec2 ProjectDistortedPoint(beam::Vec4 &X);

  /**
   * @brief Overload for pushing to output streams
   */
  friend std::ostream& operator<< (std::ostream& out, Pinhole& pinhole);

private:
  /**
   * @brief This applies the projection for to images that are not distorted
   * @return Returns image coordinates after point has been projected into image
   * plane.
   * @param X point to be projected. Not in homographic form
   */
  beam::Vec2 ApplyProjection(beam::Vec3 &X);

  /**
   * @brief This applies the projection for to images that are not distorted
   * @return Returns image coordinates after point has been projected into image
   * plane.
   * @param X point to be projected. Not in homographic form
   */
  beam::Vec2 ApplyDistortedProjection(beam::Vec3 &X);

  // Variables for storing intrinsic information
  std::string frame_id_, calibration_date_;
  bool is_K_full_ = false, is_rad_distortion_valid_ = false,
       is_tan_distortion_valid_ = false, is_calibration_date_set_ = false;
  beam::Vec2 img_dims_;
  beam::Mat3 K_;
  beam::Vec2 tan_coeffs_;
  beam::VecX rad_coeffs_ = Eigen::VectorXd::Random(3);
};

/** @} group calibration */

} // namespace beam_calibration
