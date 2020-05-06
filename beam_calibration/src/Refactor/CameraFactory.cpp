#include "beam_calibration/include/Refactor/CameraFactory.h"

namespace beam_calibration {

static std::shared_ptr<CameraModel>
    CameraFactory::Create(CameraType type, std::string& file_location,
                          uint cam_id = 0) {
  std::shared_ptr<CameraModel> model;
  if (type == CameraType::LADYBUG) {
    model = std::make_shared<LadybugCamera>(file_location, cam_id);
  } else {
    // check file type then load
    model = LoadJSON(file_location);
  }
  return model;
}

static std::shared_ptr<CameraModel>
    CameraFactory::LoadJSON(std::string& file_location) {
  BEAM_INFO("Loading file: {}", file_location);
  // load JSON
  json J;
  std::ifstream file(file_location);
  file >> J;

  int image_width = 0, image_height = 0;
  std::string camera_type, date, method, frame_id, distortion_model;
  beam::VecX coeffs, intrinsics;
  // Read values from JSON and populate variables
  camera_type = J["camera_type"];
  date = J["date"];
  method = J["method"];
  for (const auto& calib : J["calibration"]) {
    if (calib.find("distortion_model") != calib.end()) {
      distortion_model = calib["distortion_model"];
    }
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
    // push distortion coeffs (if it exists)
    if (calib.find("distortion_coefficients") != calib.end()) {
      std::vector<double> tmp_coeffs;
      for (const auto& value : calib["distortion_coefficients"]) {
        tmp_coeffs.push_back(value.get<double>());
      }
      coeffs.resize(tmp_coeffs.size());
      for (uint8_t k = 0; k < tmp_coeffs.size(); k++) {
        coeffs(k) = tmp_coeffs[k];
      }
    }
  }

  // create distortion model (if it exists)
  std::shared_ptr<DistortionModel> distortion;
  if (distortion_model == "radtan") {
    distortion = std::make_shared<RadtanDistortion>(coeffs);
  } else if (distortion_model == "equidistant") {
    distortion = std::make_shared<EquidistantDistortion>(coeffs);
  } else {
    distortion = nullptr;
  }
  // create camera model
  std::shared_ptr<CameraModel> camera;
  if (camera_type == "pinhole") {
    camera = std::make_shared<PinholeCamera>();
  } else {
    camera = nullptr;
  }
  // set values
  camera->SetIntrinsics(intrinsics);
  if (distortion) { camera->SetDistortion(distortion); }
  camera->SetImageDims(image_height, image_width);
  camera->SetFrameID(frame_id);
  camera->SetCalibrationDate(date);

  return camera;
}

} // namespace beam_calibration