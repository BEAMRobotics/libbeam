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
  if (calibration_date_ == "") { BEAM_WARN("Calibration date empty."); }
  return calibration_date_;
}

const uint32_t CameraModel::GetHeight() const {
  if (image_height_ == 0) { BEAM_WARN("Image height not set."); }
  return image_height_;
}

const uint32_t CameraModel::GetWidth() const {
  if (image_width_ == 0) { BEAM_WARN("Image width not set."); }
  return image_width_;
}

const Eigen::VectorXd CameraModel::GetIntrinsics() const {
  return intrinsics_;
}

CameraType CameraModel::GetType() const {
  return type_;
}

bool CameraModel::PixelInImage(const Eigen::Vector2i& pixel) {
  if (pixel_in[0] < 0 || pixel_in[1] < 0 || pixel_in[0] > image_width_ ||
      pixel_in[1] > image_height_)
    return false;
  return true;
}

void CameraModel::LoadJSON(const std::string& file_path) {
  BEAM_INFO("Loading file: {}", file_location);

  // load file
  json J;
  std::ifstream file(file_location);
  file >> J;

  // check type
  std::string camera_type = J["camera_type"];
  CameraType camera_type_read;
  if (camera_type == PINHOLE_RADTAN) {
    camera_type_read = CameraType::PINHOLE_RADTAN;
  } else if (camera_type == PINHOLE_EQUI) {
    camera_type_read = CameraType::PINHOLE_EQUI;
  } else if (camera_type == DOUBLESPHERE) {
    camera_type_read = CameraType::DOUBLESPHERE;
  } else {
    BEAM_CRITICAL("Invalid camera type read from json. Type read: {}. Options: "
                  "PINHOLE_RADTAN, PINHOLE_EQUI, DOUBLESPHERE",
                  camera_type.c_str());
    throw std::invalid_argument{"Invalid camera type read from json."};
  }

  if (camera_type_read != this->GetType) {
    BEAM_CRITICAL("Attempting to initialize wrong camera model type. Check "
                  "input json file.",
                  camera_type_read);
  }

  // get params
  calibration_date_ = J["date"];
  image_width_ = J["image_width"];
  image_height_ = J["image_height"];
  frame_id_ = J["frame_id"];
  std::vector intrinsics;
  for (const auto& value : J["intrinsics"]) {
    intrinsics.push_back(value.get<double>());
  }
  if (intrinsics.size() != intrinsics_size_[this->GetType]) {
    BEAM_CRITICAL("Invalid number of intrinsics read. read: {}, required: {}",
                  intrinsics.size(), intrinsics_size_[this->GetType]);
    throw std::invalid_argument{"Invalid number of instrinsics read."};
  }
}

} // namespace beam_calibration
