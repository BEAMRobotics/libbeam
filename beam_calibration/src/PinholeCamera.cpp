#include "beam_calibration/PinholeCamera.h"

namespace beam_calibration {

PinholeCamera::PinholeCamera() {
  type_ = CameraType::PINHOLE;
}

PinholeCamera::PinholeCamera(beam::VecX& intrinsics, beam::VecX& distortion,
                             uint32_t image_height, uint32_t image_width,
                             std::string frame_id, std::string date) {
  type_ = CameraType::PINHOLE;
  this->SetFrameID(frame_id);
  this->SetCalibrationDate(date);
  this->SetImageDims(image_height, image_width);
  this->SetIntrinsics(intrinsics);
  this->SetDistortionCoefficients(distortion);
}

beam::Vec2 PinholeCamera::ProjectPoint(beam::Vec3& point) {
  beam::Vec2 out_point;
  if (intrinsics_valid_ && distortion_set_) {
    // Project point
    const double fx = this->GetFx(), fy = this->GetFy(), cx = this->GetCx(),
                 cy = this->GetCy();
    const double x = point[0], y = point[1], z = point[2];
    const double rz = 1.0 / z;
    out_point << (x * rz), (y * rz);
    // Distort point using distortion model
    out_point = this->DistortPoint(out_point);
    // flip the coordinate system to be consistent with opencv convention
    double xx = out_point[0], yy = out_point[1];
    out_point[0] = (fx * (-yy) + cx);
    out_point[1] = (fy * xx + cy);
  } else if (!intrinsics_valid_) {
    LOG_ERROR("Intrinsics not set, cannot project point.");
    throw std::invalid_argument{"Intrinsics nto set"};
  }

  return out_point;
}

beam::Vec2 PinholeCamera::ProjectPoint(beam::Vec4& point) {
  bool homographic_form = (point[3] == 1);
  beam::Vec2 out_point;
  if (intrinsics_valid_ && homographic_form && distortion_set_) {
    beam::Vec3 new_point(point[0], point[1], point[2]);
    out_point = ProjectPoint(new_point);
  } else if (!intrinsics_valid_) {
    LOG_ERROR("Intrinsics not set, cannot project point.");
    throw std::invalid_argument{"Intrinsics nto set"};
  } else {
    LOG_ERROR("invalid entry, cannot project point: the point is not in "
              "homographic form, ");
    throw std::invalid_argument{"invalid point"};
  }
  return out_point;
}

beam::Vec2 PinholeCamera::DistortPoint(beam::Vec2& point) {
  beam::Vec2 coords;
  double x = point[0], y = point[1];
  beam::VecX coeffs = this->GetDistortionCoefficients();

  double xx, yy, r2, k1 = coeffs[0], k2 = coeffs[1], k3 = coeffs[2],
                     p1 = coeffs[3], p2 = coeffs[4];
  r2 = x * x + y * y;
  double quotient = (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
  xx = x * quotient + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
  yy = y * quotient + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;

  coords << xx, yy;
  return coords;
}

cv::Mat PinholeCamera::UndistortImage(cv::Mat& input_image) {
  cv::Mat output_image;
  beam::Mat3 camera_matrix = this->GetCameraMatrix();
  // convert eigen to cv mat
  cv::Mat K(3, 3, CV_8UC1);
  cv::eigen2cv(camera_matrix, K);
  // convert eigen to cv mat
  beam::VecX dist_coeffs = this->GetDistortionCoefficients();
  // opencv uses the ordering [k1, k2, r1, r2, k3]
  beam::VecX coeffs(5);
  coeffs << dist_coeffs[0], dist_coeffs[1], dist_coeffs[3], dist_coeffs[4],
      dist_coeffs[2];
  cv::Mat D(1, 5, CV_8UC1);
  cv::eigen2cv(coeffs, D);
  // undistort image
  cv::undistort(input_image, output_image, K, D);
  return output_image;
}

} // namespace beam_calibration