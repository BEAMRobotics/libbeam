#include "beam/calibration/Pinhole.h"
#include <beam/utils/log.hpp>
#include <beam/utils/math.hpp>

namespace beam_calibration {

Pinhole::Pinhole(double fx, double fy, double cx, double cy) {
  K_ << fx, 0, cx, 0, fy, cy, 0, 0, 1;
  is_full_ = true;
}

Pinhole::Pinhole(beam::Mat3 K) {
  K_ = K;
  is_full_ = true;
}

beam::Mat3 Pinhole::GetK() {
  if (!is_full_) {
    LOG_ERROR("Intrinsics matrix empty.");
  }
  return K_;
}

double Pinhole::GetFx() { return K_(0, 0); }

double Pinhole::GetFy() { return K_(1, 1); }

double Pinhole::GetCx() { return K_(0, 2); }

double Pinhole::GetCy() { return K_(1, 2); }

bool Pinhole::IsFull() {
  if (is_full_) {
    return true;
  } else {
    return false;
  }
}

void Pinhole::AddTanDist(beam::Vec2 tan_coeffs) {
  tan_coeffs_ = tan_coeffs;
  is_rad_distortion_valid_ = true;
}

void Pinhole::AddRadDist(beam::VecX rad_coeffs) {
  int rad_coeffs_size = rad_coeffs.size();
  if (rad_coeffs_size < 3 || rad_coeffs_size > 6) {
    LOG_ERROR("Invalid number of radial distortion coefficients. Input: %d, "
              "Min %d, Max: %d",
              rad_coeffs_size, 3, 6);
  } else {
    rad_coeffs_ = rad_coeffs;
    is_tan_distortion_valid_ = true;
  }
}

beam::Vec2 Pinhole::GetTanDist() {
  if (!is_tan_distortion_valid_) {
    LOG_ERROR("No tangential distortion coefficients available.");
    beam::Vec2 null_vec;
    return null_vec;
  } else {
    return tan_coeffs_;
  }
}

beam::VecX Pinhole::GetRadDist() {
  if (!is_rad_distortion_valid_) {
    LOG_ERROR("No radial distortion coefficients available.");
    beam::Vec2 null_vec;
    return null_vec;
  } else {
    return rad_coeffs_;
  }
}

beam::Vec2 Pinhole::ProjectPoint(beam::Vec3 X) {
  beam::Vec2 img_coords;
  if (is_full_) {
    img_coords = this->ApplyProjection(X);
  } else {
    LOG_ERROR("Intrinsics matrix empty, cannot project point.");
  }
  return img_coords;
}

beam::Vec2 Pinhole::ProjectPoint(beam::Vec4 X) {
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

  if (is_full_ && homographic_form) {
    img_coords = this->ApplyProjection(XX);
  } else if (!is_full_) {
    LOG_ERROR("Intrinsics matrix empty, cannot project point.");
  } else {
    LOG_ERROR("invalid entry, cannot project point: the point is not in "
              "homographic form, ");
  }

  return img_coords;
}

beam::Vec2 Pinhole::ProjectDistortedPoint(beam::Vec3 X) {
  beam::Vec2 img_coords;

  if (is_full_ && is_rad_distortion_valid_ && is_tan_distortion_valid_) {
    img_coords = this->ApplyDistortedProjection(X);
  } else if (!is_full_) {
    LOG_ERROR("Intrinsics matrix empty, cannot project point.");
  } else if (!is_rad_distortion_valid_) {
    LOG_ERROR("No radial distortion parameters, cannot project point.");
  } else if (!is_tan_distortion_valid_) {
    LOG_ERROR("No tangential distortion parameters, cannot project point.");
  }
  return img_coords;
}

beam::Vec2 Pinhole::ProjectDistortedPoint(beam::Vec4 X) {
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

  if (is_full_ && homographic_form && is_rad_distortion_valid_ &&
      is_tan_distortion_valid_) {
    img_coords = this->ApplyDistortedProjection(XX);
  } else if (!is_full_) {
    LOG_ERROR("Intrinsics matrix empty, cannot project point.");
  } else if (!is_rad_distortion_valid_) {
    LOG_ERROR("No radial distortion parameters, cannot project point.");
  } else if (!is_tan_distortion_valid_) {
    LOG_ERROR("No tangential distortion parameters, cannot project point.");
  } else {
    LOG_ERROR("invalid entry, cannot project point: the point is not in "
              "homographic form, ");
  }
  return img_coords;
}

beam::Vec2 Pinhole::ApplyProjection(beam::Vec3 X) {
  beam::Vec2 coords;
  beam::Vec3 x_proj;
  // project point
  x_proj = K_ * X;
  // normalize
  coords(0, 0) = x_proj(0, 0) / x_proj(2, 0);
  coords(1, 0) = x_proj(1, 0) / x_proj(2, 0);
  return coords;
}

beam::Vec2 Pinhole::ApplyDistortedProjection(beam::Vec3 X) {
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
  if (rad_coeffs_.size() > 3) {
    k4 = rad_coeffs_(3, 0);
  }
  if (rad_coeffs_.size() > 4) {
    k5 = rad_coeffs_(4, 0);
  }
  if (rad_coeffs_.size() > 5) {
    k6 = rad_coeffs_(5, 0);
  }

  p1 = tan_coeffs_(0, 0);
  p2 = tan_coeffs_(1, 0);

  // project point
  x_proj = K_ * X;
  x = x_proj(0, 0) / x_proj(2, 0);
  y = x_proj(1, 0) / x_proj(2, 0);
  r2 = x * x + y * y;
  xx = x * ((1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2) /
            (1 + k4 * r2 + k5 * r2 * r2 + k6 * r2 * r2 * r2)) +
       2 * x * y + p2 * (r2 + 2 * x * x);
  yy = y * ((1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2) /
            (1 + k4 * r2 + k5 * r2 * r2 + k6 * r2 * r2 * r2)) +
       p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
  coords(0, 0) = fx * xx + cx;
  coords(1, 0) = fy * yy + cy;
  return coords;
}

} // namespace beam_calibration
