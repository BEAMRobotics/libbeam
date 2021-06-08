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

std::shared_ptr<CameraModel> KannalaBrandt::Clone() {
  std::shared_ptr<KannalaBrandt> clone = std::make_shared<KannalaBrandt>();
  clone->type_ = CameraType::KANNALABRANDT;
  clone->SetIntrinsics(this->GetIntrinsics());
  clone->SetImageDims(this->GetHeight(), this->GetWidth());
  clone->SetFrameID(this->GetFrameID());
  clone->SetCalibrationDate(this->GetCalibrationDate());
  return clone;
}

bool KannalaBrandt::ProjectPoint(const Eigen::Vector3d& in_point,
                                 Eigen::Vector2d& out_pixel,
                                 bool& in_image_plane,
                                 std::shared_ptr<Eigen::MatrixXd> J) {
  double x = in_point[0], y = in_point[1], z = in_point[2];

  if (!this->InProjectionDomain(in_point)) { return false; }

  double x2 = x * x;
  double y2 = y * y;
  double z2 = z * z;
  double r = sqrt(x2 + y2);
  double th = atan(r / z);
  double th2 = th * th, th4 = th2 * th2, th6 = th4 * th2, th8 = th4 * th4;
  double th3 = th * th2, th5 = th * th4, th7 = th * th6, th9 = th * th8;

  double d = th + k1_ * th3 + k2_ * th5 + k3_ * th7 + k4_ * th9;
  out_pixel[0] = fx_ * d * (x / r) + cx_;
  out_pixel[1] = fy_ * d * (y / r) + cy_;

  if (PixelInImage(out_pixel)) {
    in_image_plane = true;
  } else {
    in_image_plane = false;
  }

  if (J) {
    /* assuming ProjectPoint(P) = [P1(x,y,z), P2(x,y,z)]^T
     *  P1(x,y,z) = fx_ * d(th) * x/r + cx_
     *  P2(x,y,z) = fy_ * d(th) * y/r + cy_
     *  Consider just P1:
     *  Take f(x,y,x) = fx_
     *  Take g(x,y,z) = d(th)
     *  Take h(x,y,z) = x/r
     *  Then P1'(x,y,z) = f'*g*h + f*g'*h + f*g*h'
     *                  = f*g'*h + f*g*h'
     */

    double g = th + k1_ * th3 + k2_ * th5 + k3_ * th7 + k4_ * th9;

    double f = fx_;
    double h = x / r;

    double dthdx = (x * z) / ((x2 + y2 + z2) * r);
    double dthdy = (y * z) / ((x2 + y2 + z2) * r);
    double dthdz = -r / (x2 + y2 + z2);

    double dgdx =
        (1 + 3 * k1_ * th2 + 5 * k2_ * th4 + 7 * k3_ * th6 + 9 * k4_ * th8) *
        dthdx;
    double dgdy =
        (1 + 3 * k1_ * th2 + 5 * k2_ * th4 + 7 * k3_ * th6 + 9 * k4_ * th8) *
        dthdy;
    double dgdz =
        (1 + 3 * k1_ * th2 + 5 * k2_ * th4 + 7 * k3_ * th6 + 9 * k4_ * th8) *
        dthdz;

    double dhdx = y2 / ((x2 + y2) * r);
    double dhdy = -(x * y) / (r * r * r);
    double dhdz = 0;

    double dP1dx = f * dgdx * h + f * g * dhdx;
    double dP1dy = f * dgdy * h + f * g * dhdy;
    double dP1dz = f * dgdz * h + f * g * dhdz;

    f = fy_;
    h = y / r;

    dhdx = -(y * x) / (r * r * r);
    dhdy = x2 / ((x2 + y2) * r);
    dhdz = 0;

    double dP2dx = f * dgdx * h + f * g * dhdx;
    double dP2dy = f * dgdy * h + f * g * dhdy;
    double dP2dz = f * dgdz * h + f * g * dhdz;

    // fill jacobian matrix
    *J << dP1dx, dP1dy, dP1dz, dP2dx, dP2dy, dP2dz;
  }
  return true;
}

bool KannalaBrandt::BackProject(const Eigen::Vector2i& in_pixel,
                                Eigen::Vector3d& out_point) {
  if (!PixelInImage(in_pixel)) { return false; }

  double u = in_pixel[0], v = in_pixel[1];
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

  out_point[0] = sin(th) * mx / ru;
  out_point[1] = sin(th) * my / ru;
  out_point[2] = cos(th);
  return true;
}

bool KannalaBrandt::InProjectionDomain(const Eigen::Vector3d& point) {
  // check pixels are valid for projection
  if (point[2] == 0) { return false; }
  return true;
}

} // namespace beam_calibration
