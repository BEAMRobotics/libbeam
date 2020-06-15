#include "beam_calibration/KannalaBrandt.h"

namespace beam_calibration {

KannalaBrandt::KannalaBrandt(const std::string& file_path) {
  type_ = CameraType::KANNALABRANDT;
  BEAM_INFO("Loading file: {}", file_path);
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

opt<Eigen::Vector2i> KannalaBrandt::ProjectPoint(const Eigen::Vector3d& point) {
  opt<Eigen::Vector2d> pixel = ProjectPointPrecise(point);
  if (pixel.has_value()) {
    Eigen::Vector2i pixel_rounded;
    pixel_rounded << std::round(pixel.value()[0]), std::round(pixel.value()[1]);
    return pixel_rounded;
  }
  return {};
}

opt<Eigen::Vector2i> KannalaBrandt::ProjectPoint(const Eigen::Vector3d& point,
                                                 Eigen::MatrixXd& J) {
  /* assuming ProjectPoint(P) = [P1(x,y,z), P2(x,y,z)]^T
   *  P1(x,y,z) = f1(x,y,z)*g1(x,y,z) + cx_
   *  P2(x,y,z) = f2(x,y,z)*g2(x,y,z) + cy_
   *  f1(x,y,z) = fxTH + fxk1TH^3 + fxk2TH^5 + fxk3TH^7 + fxk4TH^9
   *  f2(x,y,z) = fyTH + fyk1TH^3 + fyk2TH^5 + fyk3TH^7 + fyk4TH^9
   *  g1(x,y,z) = x / (sqrt(x^2 + y^2))
   *  g2(x,y,z) = y / (sqrt(x^2 + y^2))
   */
  double x = point[0], y = point[1], z = point[2];
  double r = sqrt(x * x + y * y);
  double TH = atan(r / z);
  double TH2 = TH * TH, TH4 = TH2 * TH2, TH6 = TH4 * TH2, TH8 = TH4 * TH4;
  double dTHdx = (x * z) / ((x * x + y * y + z * z) * r);
  double dTHdy = (y * z) / ((x * x + y * y + z * z) * r);
  double dTHdz = -r / (x * x + y * y + z * z);

  double g1 = x / r;
  double g2 = y / r;
  double f1 = fx_ * TH + fx_ * k1_ * TH * TH2 + fx_ * k2_ * TH * TH4 +
              fx_ * k3_ * TH * TH6 + fx_ * k4_ * TH * TH8;
  double f2 = fy_ * TH + fy_ * k1_ * TH * TH2 + fy_ * k2_ * TH * TH4 +
              fy_ * k3_ * TH * TH6 + fy_ * k4_ * TH * TH8;
  // partial derivatives for P1(x,y,z)
  double df1dx = dTHdx + (3 * k1_ * fx_ * TH2 + dTHdx) +
                 (5 * k2_ * fx_ * TH4 + dTHdx) + (7 * k3_ * fx_ * TH6 + dTHdx) +
                 (9 * k4_ * fx_ * TH8 + dTHdx);
  double df1dy = dTHdy + (3 * k1_ * fx_ * TH2 + dTHdy) +
                 (5 * k2_ * fx_ * TH4 + dTHdy) + (7 * k3_ * fx_ * TH6 + dTHdy) +
                 (9 * k4_ * fx_ * TH8 + dTHdy);
  double df1dz = dTHdz + (3 * k1_ * fx_ * TH2 + dTHdz) +
                 (5 * k2_ * fx_ * TH4 + dTHdz) + (7 * k3_ * fx_ * TH6 + dTHdz) +
                 (9 * k4_ * fx_ * TH8 + dTHdz);
  double dg1dx = (y * y) / ((x * x + y * y) * r);
  double dg1dy = -(x * y) / (r * r * r);
  double dg1dz = 0;
  double dP1dx = df1dx * g1 + dg1dx * f1;
  double dP1dy = df1dy * g1 + dg1dy * f1;
  double dP1dz = df1dz * g1 + dg1dz * f1;

  // partial derivatives for P2(x,y,z)
  double df2dx = dTHdx + (3 * k1_ * fy_ * TH2 + dTHdx) +
                 (5 * k2_ * fy_ * TH4 + dTHdx) + (7 * k3_ * fy_ * TH6 + dTHdx) +
                 (9 * k4_ * fy_ * TH8 + dTHdx);
  double df2dy = dTHdy + (3 * k1_ * fy_ * TH2 + dTHdy) +
                 (5 * k2_ * fy_ * TH4 + dTHdy) + (7 * k3_ * fy_ * TH6 + dTHdy) +
                 (9 * k4_ * fy_ * TH8 + dTHdy);
  double df2dz = dTHdz + (3 * k1_ * fy_ * TH2 + dTHdz) +
                 (5 * k2_ * fy_ * TH4 + dTHdz) + (7 * k3_ * fy_ * TH6 + dTHdz) +
                 (9 * k4_ * fy_ * TH8 + dTHdz);
  double dg2dx = -(y * x) / (r * r * r);
  double dg2dy = (x * x) / ((x * x + y * y) * r);
  double dg2dz = 0;
  double dP2dx = df2dx * g2 + dg2dx * f2;
  double dP2dy = df2dy * g2 + dg2dy * f2;
  double dP2dz = df2dz * g2 + dg2dz * f2;

  // fill jacobian matrix
  J(0, 0) = dP1dx;
  J(0, 1) = dP1dy;
  J(0, 2) = dP1dz;
  J(1, 0) = dP2dx;
  J(1, 1) = dP2dy;
  J(1, 2) = dP2dz;
  return ProjectPoint(point);
}

opt<Eigen::Vector2d>
    KannalaBrandt::ProjectPointPrecise(const Eigen::Vector3d& point) {
  // check if point is behind image plane
  if (point[2] < 0) { return {}; }

  Eigen::Vector2d out_point;
  double x = point[0], y = point[1], z = point[2];
  double Xsq_plus_Ysq = x * x + y * y;
  double theta = atan2(sqrt(Xsq_plus_Ysq), z);
  double r = sqrt(x * x + y * y);

  double theta2 = theta * theta;
  double theta3 = theta2 * theta;
  double theta5 = theta3 * theta2;
  double theta7 = theta5 * theta2;
  double theta9 = theta7 * theta2;
  double d = theta + k1_ * theta3 + k2_ * theta5 + k3_ * theta7 + k4_ * theta9;
  out_point[0] = fx_ * d * (x / r) + cx_;
  out_point[1] = fy_ * d * (y / r) + cy_;

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

} // namespace beam_calibration
