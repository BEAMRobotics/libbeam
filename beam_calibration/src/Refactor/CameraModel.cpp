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
  std::map<std::string, CameraType>::iterator it =
      intrinsics_types_.find(camera_type);
  if(it == intrinsics_types_.end()){
    BEAM_CRITICAL("Invalid camera type read from json. Type read: {}", camera_type.c_str());
    OutputCameraTypes();
    throw std::invalid_argument{"Invalid camera type read from json."};
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
  intrinsics_ = Eigen::VectorXd(intrinsics.data());
  ValidateInputs();
}

void CameraModel::OutputCameraTypes(){
  std::cout << "Intrinsic type input options:\n";
  for (std::map<std::string, CameraType>::iterator it = intrinsics_types_.start(); it != intrinsics_types_.end(); it++){
    std::cout << "    -" << it->first << "\n";
  }
}

} // namespace beam_calibration
