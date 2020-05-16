#include "beam_calibration/include/Refactor/PinholeCamera.h"

namespace beam_calibration {

beam::Vec2 PinholeCamera::ProjectPoint(beam::Vec3 point) {
  // check if point is behind image plane
  if (point[2] < 0) { return Eigen::Vector2d(-1, -1); }

  beam::Vec2 out_point;
  if (intrinsics_valid_ && distortion_set_) {
    // Project point
    const double fx = this->GetFx(), fy = this->GetFy(), cx = this->GetCx(),
                 cy = this->GetCy();
    const double x = point[0], y = point[1], z = point[2];
    const double rz = 1.0 / z;
    out_point << (x * rz), (y * rz);
    // Distort point using distortion model
    out_point = distortion_->DistortPixel(out_point);
    double xx = out_point[0], yy = out_point[1];
    out_point[0] = (fx * xx + cx);
    out_point[1] = (fy * yy + cy);
  } else if (!intrinsics_valid_) {
    BEAM_CRITICAL("intrinsics_valid_ = false");
    BEAM_CRITICAL("Intrinsics not set, cannot project point.");
    throw std::invalid_argument{"Intrinsics not set"};
  } else if (!distortion_set_) {
    BEAM_CRITICAL("Distortion not set, cannot project point.");
    throw std::invalid_argument{"Distortion not set"};
  }

  return out_point;
}

beam::Vec3 PinholeCamera::BackProject(beam::Vec2 point) {
  beam::Vec3 out_point;
  beam::Vec2 kp = point;
  kp[0] = (kp[0] - this->GetCx()) / this->GetFx();
  kp[1] = (kp[1] - this->GetCy()) / this->GetFy();
  beam::Vec2 undistorted = distortion_->UndistortPixel(kp);
  out_point << undistorted[0], (undistorted[1]), 1;
  out_point.normalize();
  return out_point;
}

beam::Vec2 PinholeCamera::ProjectUndistortedPoint(beam::Vec3 point) {
  // check if point is behind image plane
  if (point[2] < 0) { return Eigen::Vector2d(-1, -1); }

  beam::Vec2 out_point;
  if (intrinsics_valid_) {
    // Project point
    const double fx = this->GetFx(), fy = this->GetFy(), cx = this->GetCx(),
                 cy = this->GetCy();
    const double x = point[0], y = point[1], z = point[2];
    const double rz = 1.0 / z;
    out_point << (x * rz), (y * rz);
    // Distort point using distortion model
    double xx = out_point[0], yy = out_point[1];
    out_point[0] = (fx * xx + cx);
    out_point[1] = (fy * yy + cy);
  } else if (!intrinsics_valid_) {
    BEAM_CRITICAL("intrinsics_valid_ = false");
    BEAM_CRITICAL("Intrinsics not set, cannot project point.");
    throw std::invalid_argument{"Intrinsics not set"};
  } else if (!distortion_set_) {
    BEAM_CRITICAL("Distortion not set, cannot project point.");
    throw std::invalid_argument{"Distortion not set"};
  }

  return out_point;
}


void PinholeCamera::SetDistortion(std::shared_ptr<DistortionModel> model) {
  distortion_ = model;
  distortion_set_ = true;
}

beam::Mat3 CameraModel::GetCameraMatrix() const {
  beam::Mat3 K;
  K << intrinsics_[0], 0, intrinsics_[2], 0, intrinsics_[1], intrinsics_[3], 0,
      0, 1;
  return K;
}

/**
 * TODO
 */
cv::Mat PinholeCamera::UndistortImage(cv::Mat input_image) {
  if (distortion_set_) {
    return distortion_->UndistortImage(input_image);
  } else {
    BEAM_CRITICAL("Distortion not set, cannot undistort Image.");
  }
}

} // namespace beam_calibration
