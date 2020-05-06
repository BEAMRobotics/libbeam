#include "beam_calibration/include/Refactor/CameraModel.h"

namespace beam_calibration {

void CameraModel::SetFrameID(std::string id) {
  frame_id_ = id;
}

void CameraModel::SetCalibrationDate(std::string date) {
  calibration_date_ = date;
  calibration_date_set_ = true;
}

void CameraModel::SetImageDims(uint32_t height, uint32_t width) {
  image_width_ = width;
  image_height_ = height;
}

void CameraModel::SetIntrinsics(beam::VecX intrinsics) {
  if (intrinsics.size() != intrinsics_size_[this->GetType()]) {
    BEAM_CRITICAL("Invalid number of elements in intrinsics vector.");
    throw std::runtime_error{
        "Invalid number of elements in intrinsics vector."};
  } else {
    intrinsics_ = intrinsics;
    intrinsics_valid_ = true;
  }
}

void PinholeCamera::SetDistortion(std::shared_ptr<DistortionModel> model) {
  distortion_ = model;
  distortion_set_ = true;
}

const std::string CameraModel::GetFrameID() const {
  return frame_id_;
}

const std::string CameraModel::GetCalibrationDate() const {
  if (!calibration_date_set_) {
    BEAM_CRITICAL("cannot retrieve calibration date, value not set.");
    throw std::runtime_error{"cannot retrieve calibration date, value not set"};
  }
  return calibration_date_;
}

uint32_t CameraModel::GetHeight() const {
  return image_height_;
}

uint32_t CameraModel::GetWidth() const {
  return image_width_;
}

const beam::VecX CameraModel::GetIntrinsics() const {
  if (!intrinsics_valid_) {
    BEAM_CRITICAL("cannot retrieve intrinsics, value not set.");
    throw std::runtime_error{"cannot retrieve intrinsics, value not set"};
  }
  return intrinsics_;
}

CameraType CameraModel::GetType() const {
  return type_;
}

beam::Mat3 CameraModel::GetCameraMatrix() const {
  beam::Mat3 K;
  K << intrinsics_[0], 0, intrinsics_[2], 0, intrinsics_[1], intrinsics_[3], 0,
      0, 1;
  return K;
}

bool CameraModel::PixelInImage(beam::Vec2 pixel_in) {
  if (pixel_in[0] < 0 || pixel_in[1] < 0 || pixel_in[0] > this->GetWidth() ||
      pixel_in[1] > this->GetHeight())
    return false;
  return true;
}

} // namespace beam_calibration
