#include "beam_calibration/CameraModel.h"
#include "beam_calibration/DoubleSphere.h"
#include "beam_calibration/KannalaBrandt.h"
#include "beam_calibration/Ladybug.h"
#include "beam_calibration/Radtan.h"

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace beam_calibration {

std::shared_ptr<CameraModel> CameraModel::Create(std::string& file_location) {
  BEAM_INFO("Loading file: {}", file_location);
  std::shared_ptr<CameraModel> camera_model;

  std::string file_ext = boost::filesystem::extension(file_location);
  if (file_ext == ".conf") {
    camera_model = std::make_shared<Ladybug>(file_location);
  } else if (file_ext == ".json") {
    // load JSON
    json J;
    std::ifstream file(file_location);
    file >> J;
    std::string camera_type = J["camera_type"];
    if (camera_type == "KANNALABRANDT") {
      camera_model = std::make_shared<KannalaBrandt>(file_location);
    } else if (camera_type == "DOUBLESPHERE") {
      camera_model = std::make_shared<DoubleSphere>(file_location);
    } else if (camera_type == "RADTAN") {
      camera_model = std::make_shared<Radtan>(file_location);
    } else {
      BEAM_CRITICAL("Invalid camera type read from JSON.");
      throw std::runtime_error{"Invalid camera type read from JSON."};
    }
  } else {
    BEAM_CRITICAL("Invalid file type read for camera intialization.");
    throw std::runtime_error{
        "Invalid file type read for camera intialization."};
  }

  return camera_model;
}

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

void CameraModel::SetIntrinsics(const Eigen::VectorXd& intrinsics) {
  if (intrinsics.size() != intrinsics_size_[GetType()]) {
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

uint32_t CameraModel::GetHeight() const {
  if (image_height_ == 0) { BEAM_WARN("Image height not set."); }
  return image_height_;
}

uint32_t CameraModel::GetWidth() const {
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
  int max_width = image_width_, max_height = image_height_;
  if (pixel[0] < 0 || pixel[1] < 0 || pixel[0] > max_width ||
      pixel[1] > max_height)
    return false;
  return true;
}

bool CameraModel::PixelInImage(const Eigen::Vector2d& pixel) {
  double max_width = image_width_, max_height = image_height_;
  if (pixel[0] < 0 || pixel[1] < 0 || pixel[0] > max_width ||
      pixel[1] > max_height)
    return false;
  return true;
}

void CameraModel::LoadJSON(const std::string& file_location) {
  BEAM_INFO("Loading file: {}", file_location);

  // load file
  json J;
  std::ifstream file(file_location);
  file >> J;
  // get string repr of class type
  std::string class_type;
  for (std::map<std::string, CameraType>::iterator it =
           intrinsics_types_.begin();
       it != intrinsics_types_.end(); it++) {
    if (intrinsics_types_[it->first] == type_) { class_type = it->first; }
  }
  // check type
  std::string camera_type = J["camera_type"];
  std::map<std::string, CameraType>::iterator it =
      intrinsics_types_.find(camera_type);
  if (it == intrinsics_types_.end()) {
    BEAM_CRITICAL("Invalid camera type read from json. Type read: {}",
                  camera_type.c_str());
    OutputCameraTypes();
  } else if (intrinsics_types_[camera_type] != type_) {
    BEAM_CRITICAL("Camera type read from JSON does not match expected type. "
                  "Type read: {}, Expected: {}",
                  camera_type.c_str(), class_type.c_str());
  }
  // get params
  calibration_date_ = J["date"];
  image_width_ = J["image_width"];
  image_height_ = J["image_height"];
  frame_id_ = J["frame_id"];
  std::vector<double> intrinsics;
  for (const auto& value : J["intrinsics"]) {
    intrinsics.push_back(value.get<double>());
  }

  intrinsics_ = Eigen::VectorXd::Map(intrinsics.data(), intrinsics.size());
  if (intrinsics_.size() != intrinsics_size_[type_]) {
    BEAM_CRITICAL("Invalid number of intrinsics read. read: {}, required: {}",
                  intrinsics_.size(), intrinsics_size_[type_]);
    throw std::invalid_argument{"Invalid number of instrinsics read."};
  }
}

void CameraModel::OutputCameraTypes() {
  std::cout << "Intrinsic type input options:\n";
  for (std::map<std::string, CameraType>::iterator it =
           intrinsics_types_.begin();
       it != intrinsics_types_.end(); it++) {
    std::cout << "    -" << it->first << "\n";
  }
}

} // namespace beam_calibration
