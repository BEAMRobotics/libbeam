#include "beam_calibration/KannalaBrandt.h"

namespace beam_calibration {

KannalaBrandt::KannalaBrandt(const std::string& file_path) {
  type_ = CameraType::KANNALABRANDT;
  LoadJSON(file_path);
  fx_ = intrinsics_[0];
  fy_ = intrinsics_[1];
  cx_ = intrinsics_[2];
  cy_ = intrinsics_[3];
  k1_ = intrinsics_[4];
  k2_ = intrinsics_[5];
  k3_ = intrinsics_[6];
  k4_ = intrinsics_[7];
}

opt<Eigen::Vector2i>
    KannalaBrandt::ProjectPoint(const Eigen::Vector3d& point) {
  Eigen::Vector2i out_point;
  double x = point[0], y = point[1], z = point[2];
  double Xsq_plus_Ysq = point[0] * point[0] + point[1] * point[1];
  double theta = atan2(sqrt(Xsq_plus_Ysq), point[2]);

  double r = sqrt(x * x + y * y);

  double psi = atan2(point[1], point[0]);

  double theta2 = theta * theta;
  double theta3 = theta2 * theta;
  double theta5 = theta3 * theta2;
  double theta7 = theta5 * theta2;
  double theta9 = theta7 * theta2;
  double d = theta + k1_ * theta3 + k2_ * theta5 + k3_ * theta7 + k4_ * theta9;

  out_point[0] = fx_ * d * cos(psi) + cx_;
  out_point[1] = fy_ * d * sin(psi) + cy_;

  if (PixelInImage(out_point)) { return out_point; }
  return {};
}

opt<Eigen::Vector2i>
    KannalaBrandt::ProjectPoint(const Eigen::Vector3d& point,
                                Eigen::MatrixXd& J) {
  Eigen::Vector2i out_point;
  double Xsq_plus_Ysq = point[0] * point[0] + point[1] * point[1];
  double theta = atan2(sqrt(Xsq_plus_Ysq), point[2]);
  double psi = atan2(point[1], point[0]);

  double theta2 = theta * theta;
  double theta3 = theta2 * theta;
  double theta5 = theta3 * theta2;
  double theta7 = theta5 * theta2;
  double theta9 = theta7 * theta2;
  double r = theta + k1_ * theta3 + k2_ * theta5 + k3_ * theta7 + k4_ * theta9;

  out_point[0] = fx_ * r * cos(psi) + cx_;
  out_point[1] = fy_ * r * sin(psi) + cy_;

  double x = point[0], y = point[1], z = point[2];

  if (PixelInImage(out_point)) { return out_point; }
  return {};
}

opt<Eigen::Vector3d> KannalaBrandt::BackProject(const Eigen::Vector2i& pixel) {
  Eigen::Vector3d out_ray;

  double u = pixel[0], v = pixel[1];
  double mx = (u - cx_) / fx_, my = (v - cy_) / fy_;
  double ru = sqrt((mx * mx) + (my * my));

  double rth = (u - cx_) / (fx_ * mx / ru);

  // Use Newtons method to solve for theta.
  double th = rth;
  for (int i = 0; i < 5; i++) {
    double th2 = th * th;
    double th3 = th2 * th;
    double th4 = th2 * th2;
    double th6 = th4 * th2;
    double x0 = k1_ * th3 + k2_ * th4 * th + k3_ * th6 * th + k4_ * th6 * th3 -
                rth + th;
    double x1 =
        3 * k1_ * th2 + 5 * k2_ * th4 + 7 * k3_ * th6 + 9 * k4_ * th6 * th2 + 1;
    double d = 2 * x0 * x1;
    double d2 =
        4 * th * x0 *
            (3 * k1_ + 10 * k2_ * th2 + 21 * k3_ * th4 + 36 * k4_ * th6) +
        2 * x1 * x1;
    double delta = d / d2;
    th -= delta;
  }

  out_ray[0] = sin(th) * mx / ru;
  out_ray[1] = sin(th) * my / ru;
  out_ray[2] = cos(th);
  out_ray.normalize();
  return out_ray;
}

void KannalaBrandt::ValidateInputs() {
  if (intrinsics_.size() != intrinsics_size_[type_]) {
    BEAM_CRITICAL("Invalid number of intrinsics read. read: {}, required: {}",
                  intrinsics_.size(), intrinsics_size_[type_]);
    throw std::invalid_argument{"Invalid number of instrinsics read."};
  }
}

} // namespace beam_calibration
