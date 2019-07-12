#include "beam_calibration/CameraModel.h"
#include "beam_calibration/PinholeCamera.h"

using json = nlohmann::json;

namespace beam_calibration {

std::shared_ptr<CameraModel> CameraModel::LoadJSON(std::string& file_location) {
  BEAM_INFO("Loading file: {}", file_location);
  // load JSON
  json J;
  std::ifstream file(file_location);
  file >> J;

  int image_width = 0, image_height = 0;
  std::string camera_type, date, method, frame_id, distortion_model;
  beam::VecX coeffs, intrinsics, undistorted_intrinsics;
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

    if (calib.find("undistorted_intrinsics") != calib.end()) {
      std::vector<double> tmp_coeffs;
      for (const auto& value : calib["undistorted_intrinsics"]) {
        tmp_coeffs.push_back(value.get<double>());
      }
      undistorted_intrinsics.resize(tmp_coeffs.size());
      for (uint8_t k = 0; k < tmp_coeffs.size(); k++) {
        undistorted_intrinsics(k) = tmp_coeffs[k];
      }
    }
  }

  CameraType cam_type;
  DistortionType dist_type = DistortionType::NONE;
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
  std::shared_ptr<CameraModel> camera =
      CameraModel::Create(cam_type, dist_type, intrinsics, coeffs, image_height,
                          image_width, frame_id, date);
  if (undistorted_intrinsics.size() > 0) {
    camera->SetUndistortedIntrinsics(undistorted_intrinsics);
  }
  return camera;

} // namespace beam_calibration

std::shared_ptr<CameraModel>
    CameraModel::Create(CameraType type, DistortionType dist_type,
                        beam::VecX intrinsics, beam::VecX distortion,
                        uint32_t image_height, uint32_t image_width,
                        std::string frame_id, std::string date) {
  if (type == CameraType::PINHOLE) {
    return std::make_shared<PinholeCamera>(dist_type, intrinsics, distortion,
                                           image_height, image_width, frame_id,
                                           date);
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

void CameraModel::SetUndistortedIntrinsics(beam::VecX und_intrinsics) {
  if (und_intrinsics.size() != intrinsics_size_[this->GetType()]) {
    BEAM_CRITICAL("Invalid number of elements in intrinsics vector.");
    throw std::runtime_error{
        "Invalid number of elements in intrinsics vector."};
  } else {
    undistorted_intrinsics_ = und_intrinsics;
    und_intrinsics_valid_ = true;
  }
}

void CameraModel::SetDistortionCoefficients(beam::VecX distortion) {
  if (!distortion_set_) {
    BEAM_CRITICAL("Distortion has not been set");
    throw std::runtime_error{"Distortion has not been set"};
  } else if (distortion.size() != distortion_size_[this->GetDistortionType()]) {
    BEAM_CRITICAL("Invalid number of elements in coefficient vector.");
    throw std::runtime_error{
        "Invalid number of elements in coefficient vector."};
  } else {
    if (this->GetDistortionType() == DistortionType::NONE) {
      distortion = beam::VecX::Zero(5);
      BEAM_INFO("Attempting to set coefficients to non zero for 'NONE' "
                "distortion type");
    }
    distortion_coefficients_ = distortion;
    distortion_coeffs_set_ = true;
  }
}

void CameraModel::SetDistortionType(DistortionType dist) {
  distortion_set_ = true;
  beam::VecX coeffs;
  if (dist == DistortionType::RADTAN || dist == DistortionType::NONE) {
    coeffs = beam::VecX::Zero(5);
    distortion_ = std::make_unique<Radtan>();
    distortion_set_ = true;
  } else if (dist == DistortionType::EQUIDISTANT) {
    coeffs = beam::VecX::Zero(4);
    distortion_ = std::make_unique<Equidistant>();
    distortion_set_ = true;
  }
  BEAM_INFO("Distortion type changed, coefficients set to zero");
  this->SetDistortionCoefficients(coeffs);
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

const beam::VecX CameraModel::GetUndistortedIntrinsics() const {
  if (!und_intrinsics_valid_) {
    BEAM_CRITICAL("cannot retrieve intrinsics, value not set.");
    throw std::runtime_error{"cannot retrieve intrinsics, value not set"};
  }
  return undistorted_intrinsics_;
}

const beam::VecX CameraModel::GetDistortionCoefficients() const {
  if (!distortion_set_) {
    BEAM_CRITICAL("cannot retrieve distortion, value not set.");
    throw std::runtime_error{"cannot retrieve distortion, value not set"};
  }
  return distortion_coefficients_;
}

DistortionType CameraModel::GetDistortionType() const {
  if (!distortion_) {
    BEAM_CRITICAL("Distortion has not been set");
    throw std::runtime_error{"Distortion has not been set"};
  } else {
    return distortion_->GetType();
  }
}

CameraType CameraModel::GetType() const {
  return type_;
}

double CameraModel::GetFx() const {
  return intrinsics_[0];
}

double CameraModel::GetFy() const {
  return intrinsics_[1];
}

double CameraModel::GetCx() const {
  return intrinsics_[2];
}

double CameraModel::GetCy() const {
  return intrinsics_[3];
}

beam::Mat3 CameraModel::GetCameraMatrix() const {
  beam::Mat3 K;
  K << this->GetFx(), 0, this->GetCx(), 0, this->GetFy(), this->GetCy(), 0, 0,
      1;
  return K;
}

bool CameraModel::PixelInImage(beam::Vec2 pixel_in) {
  if (pixel_in[0] < 0 || pixel_in[1] < 0 || pixel_in[0] > this->GetWidth() ||
      pixel_in[1] > this->GetHeight())
    return false;
  return true;
}

/***********************Distortion Implementations**************************/

DistortionType Distortion::GetType() {
  return type_;
}

Equidistant::Equidistant() {
  type_ = DistortionType::EQUIDISTANT;
}

Radtan::Radtan() {
  type_ = DistortionType::RADTAN;
}

beam::Vec2 Radtan::DistortPixel(beam::VecX coeffs, beam::Vec2 point) const {
  beam::Vec2 coords;
  double x = point[0], y = point[1];

  double xx, yy, r2, k1 = coeffs[0], k2 = coeffs[1], k3 = coeffs[2],
                     p1 = coeffs[3], p2 = coeffs[4];
  double mx2_u = x * x;
  double my2_u = y * y;
  double mxy_u = x * y;
  double rho2_u = mx2_u + my2_u;
  double rad_dist_u =
      k1 * rho2_u + k2 * rho2_u * rho2_u + k3 * rho2_u * rho2_u * rho2_u;
  xx = x + (x * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u));
  yy = y + (y * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u));

  coords << xx, yy;
  return coords;
}

beam::Vec2 Equidistant::DistortPixel(beam::VecX coeffs,
                                     beam::Vec2 point) const {
  beam::Vec2 coords;
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
  return coords;
}

beam::Vec2 Radtan::UndistortPixel(beam::VecX coeffs, beam::Vec2 point) const {
  constexpr int n = 200; // Max. number of iterations
  beam::Vec2 y = point;
  beam::Vec2 ybar = y;
  beam::Mat2 F;
  beam::Vec2 y_tmp;
  // Handle special case around image center.
  if (y.squaredNorm() < 1e-6) return y; // Point remains unchanged.
  for (int i = 0; i < n; ++i) {
    y_tmp = DistortPixel(coeffs, ybar);
    F = ComputeJacobian(coeffs, ybar);
    beam::Vec2 e(y - y_tmp);
    beam::Vec2 du = (F.transpose() * F).inverse() * F.transpose() * e;
    ybar += du;
  }
  y = ybar;
  return y;
}

beam::Vec2 Equidistant::UndistortPixel(beam::VecX coeffs,
                                       beam::Vec2 point) const {
  constexpr int n = 30; // Max. number of iterations
  beam::Vec2 y = point;
  beam::Vec2 ybar = y;
  beam::Mat2 F;
  beam::Vec2 y_tmp;
  // Handle special case around image center.
  if (y.squaredNorm() < 1e-6) return y; // Point remains unchanged.
  for (int i = 0; i < n; ++i) {
    y_tmp = DistortPixel(coeffs, ybar);
    F = ComputeJacobian(coeffs, ybar);
    beam::Vec2 e(y - y_tmp);
    beam::Vec2 du = (F.transpose() * F).inverse() * F.transpose() * e;
    ybar += du;
  }
  y = ybar;
  return y;
}

beam::Mat2 Radtan::ComputeJacobian(beam::VecX coeffs, beam::Vec2 point) const {
  beam::Mat2 out_jacobian;
  double x = point[0];
  double y = point[1];
  double x2 = x * x;
  double y2 = y * y;
  double xy = x * y;
  const double& k1 = coeffs[0];
  const double& k2 = coeffs[1];
  const double& p1 = coeffs[3];
  const double& p2 = coeffs[4];
  double r2 = x2 + y2;
  double rad_dist_u = k1 * r2 + k2 * r2 * r2;
  const double duf_du = 1.0 + rad_dist_u + 2.0 * k1 * x2 + 4.0 * k2 * r2 * x2 +
                        2.0 * p1 * y + 6.0 * p2 * x;
  const double duf_dv =
      2.0 * k1 * xy + 4.0 * k2 * r2 * xy + 2.0 * p1 * x + 2.0 * p2 * y;
  const double dvf_du = duf_dv;
  const double dvf_dv = 1.0 + rad_dist_u + 2.0 * k1 * y2 + 4.0 * k2 * r2 * y2 +
                        2.0 * p2 * x + 6.0 * p1 * y;
  out_jacobian << duf_du, duf_dv, dvf_du, dvf_dv;
  return out_jacobian;
}

beam::Mat2 Equidistant::ComputeJacobian(beam::VecX coeffs,
                                        beam::Vec2 point) const {
  beam::Mat2 out_jacobian;
  double x = point[0];
  double y = point[1];
  double x2 = x * x;
  double y2 = y * y;
  double r = sqrt(x2 + y2);
  const double& k1 = coeffs[0];
  const double& k2 = coeffs[1];
  const double& k3 = coeffs[2];
  const double& k4 = coeffs[3];
  // Handle special case around image center.
  if (r < 1e-10) { return out_jacobian; }
  double theta = atan(r);
  double theta2 = theta * theta;
  double theta4 = theta2 * theta2;
  double theta6 = theta2 * theta4;
  double theta8 = theta4 * theta4;
  double theta3 = theta2 * theta;
  double theta5 = theta4 * theta;
  double theta7 = theta6 * theta;
  const double duf_du =
      theta * 1.0 / r *
          (k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8 + 1.0) +
      x * theta * 1.0 / r *
          ((k2 * x * theta3 * 1.0 / r * 4.0) / (x2 + y2 + 1.0) +
           (k3 * x * theta5 * 1.0 / r * 6.0) / (x2 + y2 + 1.0) +
           (k4 * x * theta7 * 1.0 / r * 8.0) / (x2 + y2 + 1.0) +
           (k1 * x * theta * 1.0 / r * 2.0) / (x2 + y2 + 1.0)) +
      ((x2) * (k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8 + 1.0)) /
          ((x2 + y2) * (x2 + y2 + 1.0)) -
      (x2)*theta * 1.0 / pow(x2 + y2, 3.0 / 2.0) *
          (k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8 + 1.0);
  const double duf_dv =
      x * theta * 1.0 / r *
          ((k2 * y * theta3 * 1.0 / r * 4.0) / (x2 + y2 + 1.0) +
           (k3 * y * theta5 * 1.0 / r * 6.0) / (x2 + y2 + 1.0) +
           (k4 * y * theta7 * 1.0 / r * 8.0) / (x2 + y2 + 1.0) +
           (k1 * y * theta * 1.0 / r * 2.0) / (x2 + y2 + 1.0)) +
      (x * y * (k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8 + 1.0)) /
          ((x2 + y2) * (x2 + y2 + 1.0)) -
      x * y * theta * 1.0 / pow(x2 + y2, 3.0 / 2.0) *
          (k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8 + 1.0);
  const double dvf_du =
      y * theta * 1.0 / r *
          ((k2 * x * theta3 * 1.0 / r * 4.0) / (x2 + y2 + 1.0) +
           (k3 * x * theta5 * 1.0 / r * 6.0) / (x2 + y2 + 1.0) +
           (k4 * x * theta7 * 1.0 / r * 8.0) / (x2 + y2 + 1.0) +
           (k1 * x * theta * 1.0 / r * 2.0) / (x2 + y2 + 1.0)) +
      (x * y * (k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8 + 1.0)) /
          ((x2 + y2) * (x2 + y2 + 1.0)) -
      x * y * theta * 1.0 / pow(x2 + y2, 3.0 / 2.0) *
          (k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8 + 1.0);
  const double dvf_dv =
      theta * 1.0 / r *
          (k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8 + 1.0) +
      y * theta * 1.0 / r *
          ((k2 * y * theta3 * 1.0 / r * 4.0) / (x2 + y2 + 1.0) +
           (k3 * y * theta5 * 1.0 / r * 6.0) / (x2 + y2 + 1.0) +
           (k4 * y * theta7 * 1.0 / r * 8.0) / (x2 + y2 + 1.0) +
           (k1 * y * theta * 1.0 / r * 2.0) / (x2 + y2 + 1.0)) +
      ((y2) * (k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8 + 1.0)) /
          ((x2 + y2) * (x2 + y2 + 1.0)) -
      (y2)*theta * 1.0 / pow(x2 + y2, 3.0 / 2.0) *
          (k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8 + 1.0);

  out_jacobian << duf_du, duf_dv, dvf_du, dvf_dv;
  return out_jacobian;
}

cv::Mat Radtan::UndistortImage(beam::Mat3 intrinsics, beam::VecX coeffs,
                               const cv::Mat& image_input, uint32_t height,
                               uint32_t width) const {
  cv::Mat output_image;
  cv::Mat K(3, 3, CV_32F);
  cv::eigen2cv(intrinsics, K);
  cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
  // convert eigen to cv mat
  beam::VecX new_coeffs(5);
  new_coeffs << coeffs[0], coeffs[1], coeffs[3], coeffs[4], coeffs[2];
  cv::Mat D(1, 5, CV_8UC1);
  cv::eigen2cv(new_coeffs, D);
  // Undistort image
  cv::Mat map1, map2;
  cv::Size img_size = cv::Size(width, height);
  cv::initUndistortRectifyMap(K, D, R, K, img_size, CV_32FC1, map1, map2);
  cv::remap(image_input, output_image, map1, map2, 1);
  return output_image;
}

cv::Mat Equidistant::UndistortImage(beam::Mat3 intrinsics, beam::VecX coeffs,
                                    const cv::Mat& image_input, uint32_t height,
                                    uint32_t width) const {
  cv::Mat output_image;
  cv::Mat K(3, 3, CV_32F);
  cv::eigen2cv(intrinsics, K);
  cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
  // convert eigen to cv mat
  cv::Mat D(1, 4, CV_8UC1);
  cv::eigen2cv(coeffs, D);
  // Undistort image
  cv::Mat map1, map2;
  cv::Size img_size = cv::Size(width, height);
  cv::Mat P;
  cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K, D, img_size, R, P);
  cv::fisheye::initUndistortRectifyMap(K, D, R, P, img_size, CV_32FC1, map1,
                                       map2);
  cv::remap(image_input, output_image, map1, map2, 1);
  return output_image;
}

} // namespace beam_calibration