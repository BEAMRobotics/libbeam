#include "beam_calibration/CameraModel.h"
#include "beam_calibration/PinholeCamera.h"

using json = nlohmann::json;

namespace beam_calibration {

CameraModel::CameraModel(beam_calibration::CameraType camera_type,
                         beam::VecX& intrinsics,
                         std::unique_ptr<DistortionModel> distortion,
                         uint32_t image_width, uint32_t image_height,
                         std::string frame_id, std::string date) {
  SetType(camera_type);
  SetFrameID(frame_id);
  SetCalibrationDate(date);
  SetImageDims(image_height, image_width);
  SetIntrinsics(intrinsics);
  SetDistortion(std::move(distortion));
}

std::shared_ptr<CameraModel> CameraModel::LoadJSON(std::string& file_location) {
  LOG_INFO("Loading file: %s", file_location.c_str());

  json J;
  int image_width = 0, image_height = 0;
  std::string camera_type, date, method, frame_id, distortion_model;
  beam::VecX coeffs;
  beam::VecX intrinsics;
  beam_calibration::CameraType cam_type;
  beam_calibration::DistortionType dist_type;

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

    std::vector<double> tmp_intrinsics;
    for (const auto& value : calib["intrinsics"]) {
      tmp_intrinsics.push_back(value.get<double>());
    }
    intrinsics.resize(tmp_intrinsics.size());
    for (uint8_t k = 0; k < tmp_intrinsics.size(); k++) {
      intrinsics(k) = tmp_intrinsics[k];
    }

    std::vector<double> tmp_coeffs;
    for (const auto& value : calib["distortion_coefficients"]) {
      tmp_coeffs.push_back(value.get<double>());
    }
    coeffs.resize(tmp_coeffs.size());
    for (uint8_t k = 0; k < tmp_coeffs.size(); k++) {
      coeffs(k) = tmp_coeffs[k];
    }
  }

  // Get type of distortion model to use
  if (distortion_model == "radtan") {
    dist_type = beam_calibration::DistortionType::RADTAN;
  } else if (distortion_model == "none") {
    dist_type = beam_calibration::DistortionType::NONE;
  } else if (distortion_model == "ladybug") {
    dist_type = beam_calibration::DistortionType::LADYBUG;
  }
  // Get type of camera model to use
  if (camera_type == "pinhole") {
    cam_type = beam_calibration::CameraType::PINHOLE;
  }

  std::unique_ptr<beam_calibration::DistortionModel> distortion =
      beam_calibration::DistortionModel::Create(dist_type, coeffs);

  std::shared_ptr<CameraModel> camera = beam_calibration::CameraModel::Create(
      cam_type, intrinsics, std::move(distortion), image_height, image_width,
      frame_id, date);

  return camera;
}

std::shared_ptr<CameraModel> CameraModel::Create(
    beam_calibration::CameraType type, beam::VecX intrinsics,
    std::unique_ptr<DistortionModel> distortion, uint32_t image_width,
    uint32_t image_height, std::string frame_id, std::string date) {
  if (type == CameraType::PINHOLE) {
    return std::shared_ptr<beam_calibration::PinholeCamera>(
        new PinholeCamera(type, intrinsics, std::move(distortion), image_width,
                          image_height, frame_id, date));
  } else {
    return nullptr;
  }
}

void CameraModel::SetFrameID(std::string id) {
  frame_id_ = id;
}

void CameraModel::SetCalibrationDate(std::string date) {
  calibration_date_ = date;
}

void CameraModel::SetImageDims(uint32_t height, uint32_t width) {
  image_width_ = width;
  image_height_ = height;
}

void CameraModel::SetIntrinsics(beam::VecX intrinsics) {
  intrinsics_ = intrinsics;
}

void CameraModel::SetDistortion(
    std::unique_ptr<beam_calibration::DistortionModel> distortion) {
  distortion_ = std::move(distortion);
}

void CameraModel::SetType(beam_calibration::CameraType type) {
  type_ = type;
}

const std::string CameraModel::GetFrameID() const {
  return frame_id_;
}

const std::string CameraModel::GetCalibrationDate() const {
  return calibration_date_;
}

beam::Vec2 CameraModel::GetImageDims() const {
  beam::Vec2 coords;
  coords << image_height_, image_width_;
  return coords;
}

const beam::VecX CameraModel::GetIntrinsics() const {
  return intrinsics_;
}

const beam_calibration::DistortionModel& CameraModel::GetDistortion() const {
  return *distortion_;
}

beam_calibration::CameraType CameraModel::GetType() {
  return type_;
}

} // namespace beam_calibration