#include "beam_calibration/Pinhole.h"

using json = nlohmann::json;

namespace beam_calibration {

Pinhole::Pinhole(double& fx, double& fy, double& cx, double& cy) {
  K_ << fx, 0, cx, 0, fy, cy, 0, 0, 1;
  is_K_full_ = true;
}

Pinhole::Pinhole(beam::Mat3& K) {
  if (K(0, 0) != 0 && K(0, 1) == 0 && K(0, 2) != 0 && K(1, 0) == 0 &&
      K(1, 1) != 0 && K(1, 2) != 0 && K(2, 0) == 0 && K(2, 1) == 0 &&
      K(2, 2) == 1) {
    K_ = K;
    is_K_full_ = true;
  } else {
    LOG_ERROR("Invalid projection matrix (K) input.");
  }
}

void Pinhole::LoadJSON(std::string& file_location) {
  LOG_INFO("Loading file: %s", file_location.c_str());

  json J;
  int counter = 0, image_width, image_height;
  std::string type, date, method, frame_id, distortion_model;
  beam::Vec2 tan_coeffs, img_dims;
  beam::VecX rad_coeffs;
  beam::Mat3 K;

  std::ifstream file(file_location);
  file >> J;

  type = J["type"];
  date = J["date"];
  method = J["method"];

  if (type != "pinhole_calibration") {
    LOG_ERROR(
        "Attempting to create Pinhole object with invalid json type. Type: %s",
        type.c_str());
    throw std::invalid_argument{
        "Attempting to create Pinhole object invalid json type"};
    return;
  }

  SetCalibrationDate(date);

  LOG_INFO("Type: %s", type.c_str());
  LOG_INFO("Date: %s", date.c_str());
  LOG_INFO("Method: %s", method.c_str());

  for (const auto& calib : J["calibration"]) {
    int i = 0, j = 0;

    image_width = calib["image_width"].get<int>();
    image_height = calib["image_height"].get<int>();
    img_dims <<  image_width, image_height;
    frame_id = calib["frame_id"];

    for (const auto& Kij : calib["camera_matrix"]) {
      counter++;
      K(i, j) = Kij.get<double>();
      if (j == 2) {
        i++;
        j = 0;
      } else {
        j++;
      }
    }
    if (counter != 9) {
      LOG_ERROR("Invalid camera_matrix in .json file.");
      throw std::invalid_argument{"Invalid camera_matrix in .json file."};
      return;
    }

    distortion_model = calib["distortion_model"];
    if(distortion_model != "radtan"){
      LOG_ERROR("Invalid distortion model in json file. Model: %s",
                distortion_model.c_str());
      throw std::invalid_argument{"Invalid distortion model in json file."};
      return;
    }

    std::vector<double> rad_tmp;
    for (const auto& value : calib["rad_coeffs"]) {
      rad_tmp.push_back(value.get<double>());
    }

    rad_coeffs.resize(rad_tmp.size());
    for (uint8_t k = 0; k<rad_tmp.size(); k++){
      rad_coeffs(k) = rad_tmp[k];
    }

    counter = 0;
    for (const auto& value : calib["tan_coeffs"]) {
      if(counter !=2){
        tan_coeffs(counter, 1) = value.get<double>();
      } else {
        LOG_ERROR("Too many tangential coefficients in json file");
        throw std::invalid_argument{"Too many tangential coefficients in json file"};
        return;
      }
      counter++;
    }
  }

  // build object
  SetFrameId(frame_id);
  SetImgDims(img_dims);
  SetK(K);
  SetCalibrationDate(date);
  SetTanDist(tan_coeffs);
  SetRadDist(rad_coeffs);
}

void Pinhole::SetFrameId(std::string& frame_id) {
  frame_id_ = frame_id;
}

std::string Pinhole::GetFrameId() {
  return frame_id_;
}

void Pinhole::SetImgDims(beam::Vec2& img_dims) {
  img_dims_ = img_dims;
}

beam::Vec2 Pinhole::GetImgDims() {
  return img_dims_;
}

void Pinhole::SetK(beam::Mat3 K){
  if (is_K_full_)
  {
    LOG_ERROR("Cannot add camera matrix, value already exists.");
    throw std::runtime_error{"Cannot add camera matrix, value already exists."};
    return;
  } else {
    K_ = K;
    is_K_full_ = true;
  }
}

beam::Mat3 Pinhole::GetK() {
  if (!is_K_full_) {
    LOG_ERROR("Intrinsics matrix empty.");
    throw std::invalid_argument{"no intrinsics matrix"};
  }
  return K_;
}

double Pinhole::GetFx() {
  return K_(0, 0);
}

double Pinhole::GetFy() {
  return K_(1, 1);
}

double Pinhole::GetCx() {
  return K_(0, 2);
}

double Pinhole::GetCy() {
  return K_(1, 2);
}

void Pinhole::SetCalibrationDate(std::string& calibration_date) {
  calibration_date_ = calibration_date;
  is_calibration_date_set_ = true;
}

std::string Pinhole::GetCalibrationDate() {
  if (!is_calibration_date_set_) {
    throw std::runtime_error{"cannot retrieve calibration date, value not set"};
    LOG_ERROR("cannot retrieve calibration date, value not set.");
  }
  return calibration_date_;
}

bool Pinhole::IsKFull() {
  if (is_K_full_) {
    return true;
  } else {
    return false;
  }
}

void Pinhole::SetTanDist(beam::Vec2& tan_coeffs) {
  tan_coeffs_ = tan_coeffs;
  is_tan_distortion_valid_ = true;
}

void Pinhole::SetRadDist(beam::VecX rad_coeffs) {
  is_rad_distortion_valid_ = false;
  int rad_coeffs_size = rad_coeffs.size();
  if (rad_coeffs_size < 3 || rad_coeffs_size > 6) {
    LOG_ERROR("Invalid number of radial distortion coefficients. Input: %d, "
              "Min %d, Max: %d",
              rad_coeffs_size, 3, 6);
  } else {
    // rad_coeffs_(rad_coeffs.size());
    rad_coeffs_ = rad_coeffs;
    is_rad_distortion_valid_ = true;
  }
}

beam::Vec2 Pinhole::GetTanDist() {
  if (!is_tan_distortion_valid_) {
    LOG_ERROR("No tangential distortion coefficients available.");
    beam::Vec2 null_vec;
    throw std::invalid_argument{
        "invalid/non-existing pinhole tangential distortion param"};
    return null_vec;
  } else {
    return tan_coeffs_;
  }
}

beam::VecX Pinhole::GetRadDist() {
  if (!is_rad_distortion_valid_) {
    LOG_ERROR("No radial distortion coefficients available.");
    beam::Vec2 null_vec;
    throw std::invalid_argument{
        "invalid/non-existing pinhole radial distortion param"};
    return null_vec;
  } else {
    return rad_coeffs_;
  }
}

beam::Vec2 Pinhole::ProjectPoint(beam::Vec3& X) {
  beam::Vec2 img_coords;
  if (is_K_full_) {
    img_coords = this->ApplyProjection(X);
  } else {
    LOG_ERROR("Intrinsics matrix empty, cannot project point.");
    throw std::invalid_argument{"no intrinsics matrix"};
  }
  return img_coords;
}

beam::Vec2 Pinhole::ProjectPoint(beam::Vec4& X) {
  beam::Vec2 img_coords;
  beam::Vec3 XX;
  bool homographic_form;

  if (X(3, 0) == 1) {
    homographic_form = true;
    XX(0, 0) = X(0, 0);
    XX(1, 0) = X(1, 0);
    XX(2, 0) = X(2, 0);
  } else {
    homographic_form = false;
  }

  if (is_K_full_ && homographic_form) {
    img_coords = this->ApplyProjection(XX);
  } else if (!is_K_full_) {
    LOG_ERROR("Intrinsics matrix empty, cannot project point.");
    throw std::invalid_argument{"no intrinsics matrix"};
  } else {
    LOG_ERROR("invalid entry, cannot project point: the point is not in "
              "homographic form, ");
    throw std::invalid_argument{"invalid point"};
  }
  return img_coords;
}

beam::Vec2 Pinhole::ProjectDistortedPoint(beam::Vec3& X) {
  beam::Vec2 img_coords;

  if (is_K_full_ && is_rad_distortion_valid_ && is_tan_distortion_valid_) {
    img_coords = this->ApplyDistortedProjection(X);
  } else if (!is_K_full_) {
    LOG_ERROR("Intrinsics matrix empty, cannot project point.");
    throw std::invalid_argument{"no intrinsics matrix"};
  } else if (!is_rad_distortion_valid_) {
    LOG_ERROR("No radial distortion parameters, cannot project point.");
    throw std::invalid_argument{
        "invalid/non-existing pinhole radial distortion param"};
  } else if (!is_tan_distortion_valid_) {
    LOG_ERROR("No tangential distortion parameters, cannot project point.");
    throw std::invalid_argument{
        "invalid/non-existing pinhole tangential distortion param"};
  }
  return img_coords;
}

beam::Vec2 Pinhole::ProjectDistortedPoint(beam::Vec4& X) {
  beam::Vec2 img_coords;
  beam::Vec3 XX;

  bool homographic_form;
  if (X(3, 0) == 1) {
    homographic_form = true;
    XX(0, 0) = X(0, 0);
    XX(1, 0) = X(1, 0);
    XX(2, 0) = X(2, 0);
  } else {
    homographic_form = false;
  }

  if (is_K_full_ && homographic_form && is_rad_distortion_valid_ &&
      is_tan_distortion_valid_) {
    img_coords = this->ApplyDistortedProjection(XX);
  } else if (!is_K_full_) {
    LOG_ERROR("Intrinsics matrix empty, cannot project point.");
    throw std::invalid_argument{"no intrinsics matrix"};
  } else if (!is_rad_distortion_valid_) {
    LOG_ERROR("No radial distortion parameters, cannot project point.");
    throw std::invalid_argument{
        "invalid/non-existing pinhole radial distortion param"};
  } else if (!is_tan_distortion_valid_) {
    LOG_ERROR("No tangential distortion parameters, cannot project point.");
    throw std::invalid_argument{
        "invalid/non-existing pinhole tangential distortion param"};
  } else {
    LOG_ERROR("invalid entry, cannot project point: the point is not in "
              "homographic form, ");
    throw std::invalid_argument{"invalid point"};
  }
  return img_coords;
}

beam::Vec2 Pinhole::ApplyProjection(beam::Vec3& X) {
  beam::Vec2 coords;
  beam::Vec3 x_proj;
  // project point
  x_proj = K_ * X;
  // normalize
  coords(0, 0) = x_proj(0, 0) / x_proj(2, 0);
  coords(1, 0) = x_proj(1, 0) / x_proj(2, 0);
  return coords;
}

beam::Vec2 Pinhole::ApplyDistortedProjection(beam::Vec3& X) {
  beam::Vec2 coords;
  beam::Vec3 x_proj;
  double x, y, xx, yy, r2, fx, fy, cx, cy, k1, k2, k3, k4 = 0, k5 = 0, k6 = 0,
                                                       p1, p2;
  // get focal length and camera center:
  fx = this->GetFx();
  fy = this->GetFy();
  cx = this->GetCx();
  cy = this->GetCy();

  // get distortion coeffs:
  k1 = rad_coeffs_(0, 0);
  k2 = rad_coeffs_(1, 0);
  k3 = rad_coeffs_(2, 0);
  if (rad_coeffs_.size() > 3) { k4 = rad_coeffs_(3, 0); }
  if (rad_coeffs_.size() > 4) { k5 = rad_coeffs_(4, 0); }
  if (rad_coeffs_.size() > 5) { k6 = rad_coeffs_(5, 0); }

  p1 = tan_coeffs_(0, 0);
  p2 = tan_coeffs_(1, 0);

  // project point
  x = X(0, 0) / X(2, 0);
  y = X(1, 0) / X(2, 0);
  r2 = x * x + y * y;
  double quotient = (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2) /
                    (1 + k4 * r2 + k5 * r2 * r2 + k6 * r2 * r2 * r2);
  xx = x * quotient + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
  yy = y * quotient + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
  coords(0, 0) = fx * xx + cx;
  coords(1, 0) = fy * yy + cy;
  return coords;
}

std::ostream& operator<<(std::ostream& out, Pinhole& pinhole){
  out << "Calibration date: " << pinhole.GetCalibrationDate() << std::endl;
  out << "Frame ID: " << pinhole.GetFrameId() << std::endl;
  out << "Image center: [cx, cy] = [" << pinhole.GetCx() << ", " << pinhole.GetCy() << "]." << std::endl;
  out << "Focal length: [fx, fy] = [" << pinhole.GetFx() << ", " << pinhole.GetFy() << "]." << std::endl;
  out << "Image dimensions: [w, h] = [" << pinhole.GetImgDims()[0] << ", " << pinhole.GetImgDims()[1] << "]." << std::endl;
  return out;
}

} // namespace beam_calibration
