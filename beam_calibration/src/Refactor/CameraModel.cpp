#include "beam_calibration/include/Refactor/CameraModel.h"

namespace beam_calibration {

void CameraModel::SetFrameID(const std::string& id) {
  frame_id_ = id;
}

void CameraModel::SetCalibrationDate(const std::string& date) {
  calibration_date_ = date;
}

void CameraModel::SetImageDims(const uint32_t height, const uint32_t width) {
  image_width_ = width;
  image_height_ = height;
}

void CameraModel::SetIntrinsics(const Eigen::VectorXd& instrinsics) {
  if (intrinsics.size() != intrinsics_size_[this->GetType()]) {
    BEAM_CRITICAL("Invalid number of elements in intrinsics vector.");
    throw std::runtime_error{
        "Invalid number of elements in intrinsics vector."};
  } else {
    intrinsics_ = intrinsics;
  }
}

const std::string CameraModel::GetFrameID() const {
  return frame_id_;
}

const std::string CameraModel::GetCalibrationDate() const {
  if (calibration_date_ == "") {
    BEAM_WARN("Calibration date empty.");
  }
  return calibration_date_;
}

const uint32_t CameraModel::GetHeight() const {
  if (image_height_ == 0) {
    BEAM_WARN("Image height not set.");
  }
  return image_height_;
}

const uint32_t CameraModel::GetWidth() const {
  if (image_width_ == 0) {
    BEAM_WARN("Image width not set.");
  }
  return image_width_;
}

const Eigen::VectorXd CameraModel::GetIntrinsics() const {
  return intrinsics_;
}

CameraType CameraModel::GetType() const {
  return type_;
}

bool CameraModel::PixelInImage(const Eigen::Vector3d& pixel) {
  if (pixel_in[0] < 0 || pixel_in[1] < 0 || pixel_in[0] > image_width_ ||
      pixel_in[1] > image_height_)
    return false;
  return true;
}

} // namespace beam_calibration
