#include "beam_calibration/CameraModel.h"
#include "beam_calibration/EquidistantCamera.h"
#include "beam_calibration/PinholeCamera.h"

using json = nlohmann::json;

namespace beam_calibration {

std::shared_ptr<CameraModel> CameraModel::LoadJSON(std::string& file_location) {
  LOG_INFO("Loading file: %s", file_location.c_str());

  json J;
  int image_width = 0, image_height = 0;
  std::string camera_type, date, method, frame_id, distortion_model;
  beam::VecX coeffs, intrinsics;
  CameraType cam_type = CameraType::NONE;

  std::ifstream file(file_location);
  file >> J;

  camera_type = J["camera_type"];
  date = J["date"];
  method = J["method"];
  for (const auto& calib : J["calibration"]) {
    distortion_model = calib["distortion_model"];
    image_width = calib["image_width"].get<int>();
    image_height = calib["image_height"].get<int>();
    frame_id = calib["frame_id"];
    // push intrinsics
    std::vector<double> tmp_intrinsics;
    for (const auto& value : calib["intrinsics"]) {
      tmp_intrinsics.push_back(value.get<double>());
    }
    intrinsics.resize(tmp_intrinsics.size());
    for (uint8_t k = 0; k < tmp_intrinsics.size(); k++) {
      intrinsics(k) = tmp_intrinsics[k];
    }
    // push distortion coeffs
    std::vector<double> tmp_coeffs;
    for (const auto& value : calib["distortion_coefficients"]) {
      tmp_coeffs.push_back(value.get<double>());
    }
    coeffs.resize(tmp_coeffs.size());
    for (uint8_t k = 0; k < tmp_coeffs.size(); k++) {
      coeffs(k) = tmp_coeffs[k];
    }
  }

  // Get type of camera model to use
  if (camera_type == "pinhole") {
    cam_type = CameraType::PINHOLE;
  } else if (camera_type == "fisheye") {
    // if its fisheye, create camera based off distortion it uses
    if (distortion_model == "equidistant") {
      cam_type = CameraType::EQUIDISTANT;
    }
  }

  // create camera model
  std::shared_ptr<CameraModel> camera = CameraModel::Create(
      cam_type, intrinsics, coeffs, image_height, image_width, frame_id, date);

  return camera;
}

std::shared_ptr<CameraModel>
    CameraModel::Create(CameraType type, beam::VecX intrinsics,
                        beam::VecX distortion, uint32_t image_height,
                        uint32_t image_width, std::string frame_id,
                        std::string date) {
  if (type == CameraType::PINHOLE) {
    return std::shared_ptr<PinholeCamera>(new PinholeCamera(
        intrinsics, distortion, image_height, image_width, frame_id, date));
  } else if (type == CameraType::EQUIDISTANT) {
    return std::shared_ptr<EquidistantCamera>(new EquidistantCamera(
        intrinsics, distortion, image_height, image_width, frame_id, date));
  } else {
    return nullptr;
  }
}

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

void CameraModel::SetIntrinsics(beam::VecX& intrinsics) {
  // throw error if input isnt correct size
  if (intrinsics.size() != 4) {
    LOG_ERROR("Invalid number of elements in intrinsics vector.");
    throw std::runtime_error{
        "Invalid number of elements in intrinsics vector."};
  } else {
    intrinsics_ = intrinsics;
    intrinsics_valid_ = true;
  }
}

void CameraModel::SetDistortionCoefficients(beam::VecX& distortion) {
  if (distortion.size() != get_coeffs_size_[this->GetType()]) {
    LOG_ERROR("Invalid number of elements in coefficient vector.");
    throw std::runtime_error{
        "Invalid number of elements in coefficient vector."};
  } else {
    distortion_coefficients_ = distortion;
    distortion_set_ = true;
  }
}

const std::string CameraModel::GetFrameID() const {
  return frame_id_;
}

const std::string CameraModel::GetCalibrationDate() const {
  if (!calibration_date_set_) {
    LOG_ERROR("cannot retrieve calibration date, value not set.");
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

const beam::VecX& CameraModel::GetIntrinsics() const {
  if (!intrinsics_valid_) {
    LOG_ERROR("cannot retrieve intrinsics, value not set.");
    throw std::runtime_error{"cannot retrieve intrinsics, value not set"};
  }
  return intrinsics_;
}

const beam::VecX& CameraModel::GetDistortionCoefficients() const {
  if (!distortion_set_) {
    LOG_ERROR("cannot retrieve distortion, value not set.");
    throw std::runtime_error{"cannot retrieve distortion, value not set"};
  }
  return distortion_coefficients_;
}

CameraType CameraModel::GetType() {
  return type_;
}

double CameraModel::GetFx() {
  return intrinsics_[0];
}

double CameraModel::GetFy() {
  return intrinsics_[1];
}

double CameraModel::GetCx() {
  return intrinsics_[2];
}

double CameraModel::GetCy() {
  return intrinsics_[3];
}

beam::Mat3 CameraModel::GetCameraMatrix() {
  beam::Mat3 K;
  K << this->GetFx(), 0, this->GetCx(), 0, this->GetFy(), this->GetCy(), 0, 0,
      1;
  return K;
}

} // namespace beam_calibration