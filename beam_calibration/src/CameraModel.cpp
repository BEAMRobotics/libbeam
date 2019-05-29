#include "beam_calibration/CameraModel.h"
#include "beam_calibration/PinholeCamera.h"

using json = nlohmann::json;

namespace beam_calibration {

std::shared_ptr<CameraModel> CameraModel::LoadJSON(std::string& file_location) {
  LOG_INFO("Loading file: %s", file_location.c_str());
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

  CameraType cam_type;
  DistortionType dist_type;
  // Get type of camera model and distortion model to use
  if (camera_type == "pinhole") {
    cam_type = CameraType::PINHOLE;
    if (distortion_model == "radtan") {
      dist_type = DistortionType::RADTAN;
    } else if (distortion_model == "equidistant") {
      dist_type = DistortionType::EQUIDISTANT;
    }
  }

  // create camera model
  return CameraModel::Create(cam_type, dist_type, intrinsics, coeffs,
                             image_height, image_width, frame_id, date);

} // namespace beam_calibration

std::shared_ptr<CameraModel>
    CameraModel::Create(CameraType type, DistortionType dist_type,
                        beam::VecX intrinsics, beam::VecX distortion,
                        uint32_t image_height, uint32_t image_width,
                        std::string frame_id, std::string date) {
  if (type == CameraType::PINHOLE) {
    return std::shared_ptr<PinholeCamera>(
        new PinholeCamera(dist_type, intrinsics, distortion, image_height,
                          image_width, frame_id, date));
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
  if (intrinsics.size() != intrinsics_size_[this->GetType()]) {
    LOG_ERROR("Invalid number of elements in intrinsics vector.");
    throw std::runtime_error{
        "Invalid number of elements in intrinsics vector."};
  } else {
    intrinsics_ = intrinsics;
    intrinsics_valid_ = true;
  }
}

void CameraModel::SetDistortionCoefficients(beam::VecX& distortion) {
  if (!distortion_set_) {
    LOG_ERROR("Distortion has not been set");
    throw std::runtime_error{"Distortion has not been set"};
  } else if (distortion.size() != distortion_size_[this->GetDistortionType()]) {
    LOG_ERROR("Invalid number of elements in coefficient vector.");
    throw std::runtime_error{
        "Invalid number of elements in coefficient vector."};
  } else {
    distortion_coefficients_ = distortion;
    distortion_coeffs_set_ = true;
  }
}

void CameraModel::SetDistortionType(DistortionType dist) {
  distortion_ = std::unique_ptr<Distortion>(new Distortion(dist));
  distortion_set_ = true;
  beam::VecX dist;
  if (this->GetDistortionType == DistortionType::RADTAN) {
    dist = beam::VecX::Zero(5);
  } else if (this->GetDistortionType == DistortionType::EQUIDISTANT) {
    dist = beam::VecX::Zero(4);
  }
  this->SetDistortionCoefficients(dist);
  LOG_INFO("Distortion coefficients set to zero due to new distortion type");
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

DistortionType CameraModel::GetDistortionType() {
  if (!distortion_) {
    LOG_ERROR("Distortion has not been set");
    throw std::runtime_error{"Distortion has not been set"};
  } else {
    return distortion_->GetType();
  }
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

Distortion::Distortion(DistortionType type) {
  type_ = type;
}

DistortionType Distortion::GetType() {
  return type_;
}

beam::Vec2 Distortion::Distort(beam::VecX coeffs, beam::Vec2 point) {
  beam::Vec2 coords;
  if (type_ == DistortionType::RADTAN) {
    double x = point[0], y = point[1];

    double xx, yy, r2, k1 = coeffs[0], k2 = coeffs[1], k3 = coeffs[2],
                       p1 = coeffs[3], p2 = coeffs[4];
    r2 = x * x + y * y;
    double quotient = (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
    xx = x * quotient + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
    yy = y * quotient + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;

    coords << xx, yy;
  } else if (type_ == DistortionType::EQUIDISTANT) {
    double x = point[0], y = point[1];

    const double k1 = coeffs[0], k2 = coeffs[1], k3 = coeffs[2], k4 = coeffs[3];

    double x2 = x * x;
    double y2 = y * y;
    double r = sqrt(x2 + y2);

    double theta = atan(r);
    double theta2 = theta * theta;
    double theta4 = theta2 * theta2;
    double theta6 = theta2 * theta4;
    double theta8 = theta4 * theta4;
    double thetad =
        theta * (1 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);

    double scaling = (r > 1e-8) ? thetad / r : 1.0;
    x *= scaling;
    y *= scaling;

    coords << x, y;
  } else if (type_ == DistortionType::NONE) {
    return point;
  }
  return coords;
}

cv::Mat Distortion::UndistortImage(beam::Mat3 intrinsics, beam::VecX coeffs,
                                   cv::Mat& image_input, uint32_t height,
                                   uint32_t width) {
  cv::Mat output_image;
  cv::Mat K(3, 3, CV_32F);
  cv::eigen2cv(intrinsics, K);
  cv::Mat R = cv::Mat::eye(3, 3, CV_32F);

  if (type_ == DistortionType::RADTAN) {
    // convert eigen to cv mat
    beam::VecX new_coeffs(5);
    new_coeffs << coeffs[0], coeffs[1], coeffs[3], coeffs[4], coeffs[2];
    cv::Mat D(1, 5, CV_8UC1);
    cv::eigen2cv(new_coeffs, D);
    // undistort image
    cv::Mat map1, map2;
    cv::Size img_size = cv::Size(width, height);
    cv::initUndistortRectifyMap(K, D, R, K, img_size, CV_32FC1, map1, map2);
    cv::remap(image_input, output_image, map1, map2, 1);
  } else if (type_ == DistortionType::EQUIDISTANT) {
    cv::Mat P = K.clone();
    P.at<double>(0, 0) = K.at<double>(0, 0) / 1.5;
    P.at<double>(1, 1) = K.at<double>(1, 1) / 1.5;
    // convert eigen to cv mat
    cv::Mat D(4, 1, CV_8UC1);
    cv::eigen2cv(coeffs, D);
    // undistort image
    cv::Mat map1, map2;
    cv::Size img_size = cv::Size(width, height);
    cv::fisheye::initUndistortRectifyMap(K, D, R, P, img_size, CV_32FC1, map1,
                                         map2);
    cv::remap(image_input, output_image, map1, map2, 1);
  } else if (type_ == DistortionType::NONE) {
    return image_input;
  }
  return output_image;
}

} // namespace beam_calibration