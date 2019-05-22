#include "beam_calibration/LadybugCamera.h"

namespace beam_calibration {

beam::Vec2 LadybugCamera::ProjectPoint(beam::Vec3& point) {
  beam::Vec2 out_point;
  // Project point
  const double fx = intrinsics_[0], fy = intrinsics_[1], cx = intrinsics_[2],
               cy = intrinsics_[3];
  const double x = point[0], y = point[1], z = point[2];
  const double rz = 1.0 / z;
  out_point << (x * rz), (y * rz);
  // Distort point
  out_point = distortion_->Distort(out_point);
  // flip the coordinate system to be consistent with opencv convention shown:
  // http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/OWENS/LECT9/node2.html
  double xx = out_point[0], yy = out_point[1];
  out_point[0] = (fx * (-yy) + cx);
  out_point[1] = (fy * xx + cy);

  return out_point;
}

beam::Vec2 LadybugCamera::ProjectPoint(beam::Vec4& point) {}
} // namespace beam_calibration