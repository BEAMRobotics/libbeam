#include "beam_calibration/PinholeCamera.h"

namespace beam_calibration {

PinholeCamera::PinholeCamera() {
  type_ = CameraType::PINHOLE;
}

PinholeCamera::PinholeCamera(DistortionType dist_type, beam::VecX intrinsics,
                             beam::VecX distortion, uint32_t image_height,
                             uint32_t image_width, std::string frame_id,
                             std::string date) {
  type_ = CameraType::PINHOLE;
  this->SetDistortionType(dist_type);
  this->SetFrameID(frame_id);
  this->SetCalibrationDate(date);
  this->SetImageDims(image_height, image_width);
  this->SetIntrinsics(intrinsics);
  this->SetDistortionCoefficients(distortion);
}

beam::Vec2 PinholeCamera::ProjectPoint(beam::Vec3 point) {
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

beam::Vec2 PinholeCamera::ProjectUndistortedPoint(beam::Vec3 point) {
  beam::Vec2 out_point;
  if (und_intrinsics_valid_) {
    // Project point
    beam::VecX intrinsics = this->GetUndistortedIntrinsics();
    const double fx = intrinsics[0], fy = intrinsics[1], cx = intrinsics[2],
                 cy = intrinsics[3];
    const double x = point[0], y = point[1], z = point[2];
    const double rz = 1.0 / z;
    out_point << (x * rz), (y * rz);
    double xx = out_point[0], yy = out_point[1];
    out_point[0] = (fx * xx + cx);
    out_point[1] = (fy * yy + cy);
  } else if (!und_intrinsics_valid_) {
    BEAM_CRITICAL("und_intrinsics_valid_ = false");
    BEAM_CRITICAL("Intrinsics not set, cannot project point.");
    throw std::invalid_argument{"Intrinsics not set"};
  }
  return out_point;
}

beam::Vec2 PinholeCamera::ProjectPoint(beam::Vec4 point) {
  bool homographic_form = (point[3] == 1);
  beam::Vec2 out_point;
  if (intrinsics_valid_ && homographic_form && distortion_set_) {
    beam::Vec3 new_point(point[0], point[1], point[2]);
    out_point = ProjectPoint(new_point);
  } else if (!intrinsics_valid_) {
    BEAM_CRITICAL("intrinsics_valid_ = false");
    BEAM_CRITICAL("Intrinsics not set, cannot project point.");
    throw std::invalid_argument{"Intrinsics not set"};
  } else {
    BEAM_CRITICAL("invalid entry, cannot project point: the point is not in "
                  "homographic form, ");
    throw std::invalid_argument{"invalid point"};
  }
  return out_point;
}

beam::Vec2 PinholeCamera::DistortPoint(beam::Vec2 point) {
  beam::VecX distortion_coeffs = this->GetDistortionCoefficients();
  return distortion_->DistortPixel(distortion_coeffs, point);
}

beam::Vec2 PinholeCamera::UndistortPoint(beam::Vec2 point) {
  beam::VecX distortion_coeffs = this->GetDistortionCoefficients();
  return distortion_->UndistortPixel(distortion_coeffs, point);
}

cv::Mat PinholeCamera::UndistortImage(cv::Mat input_image) {
  beam::Mat3 camera_matrix = this->GetCameraMatrix();
  beam::VecX distortion_coeffs = this->GetDistortionCoefficients();
  return distortion_->UndistortImage(camera_matrix, distortion_coeffs,
                                     input_image, this->GetHeight(),
                                     this->GetWidth());
}

beam::Vec3 PinholeCamera::BackProject(beam::Vec2 point) {
  beam::Vec3 out_point;
  beam::Vec2 kp = point;
  kp[0] = (kp[0] - this->GetCx()) / this->GetFx();
  kp[1] = (kp[1] - this->GetCy()) / this->GetFy();
  beam::Vec2 undistorted =
      distortion_->UndistortPixel(this->GetDistortionCoefficients(), kp);
  out_point << undistorted[0], (undistorted[1]), 1;
  out_point.normalize();
  return out_point;
}

} // namespace beam_calibration
