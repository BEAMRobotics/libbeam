#include "beam_calibration/RadTanDistortion.h"

namespace beam_calibration {

RadTanDistortion::RadTanDistortion(beam::VecX coeffs) {
  SetCoefficients(coeffs);
}

beam::Vec2 RadTanDistortion::Distort(beam::Vec2& point) {
  beam::Vec2 distorted_point;
  double x = point[0], y = point[1];
  beam::VecX coeffs = GetCoefficients();
  const double k1 = coeffs[0], k2 = coeffs[1], k3 = coeffs[2], p1 = coeffs[3],
               p2 = coeffs[4];
  double x2 = x * x, y2 = y * y, xy = x * y, r2 = x2 * y2;
  double rad_dist = k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
  x += x * rad_dist + 2.0 * p1 * xy + p2 * (r2 + 2.0 * x2);
  y += y * rad_dist + 2.0 * p2 * xy + p1 * (r2 + 2.0 * y2);
  distorted_point << x, y;
  return distorted_point;
}

beam::Vec2 RadTanDistortion::Undistort(beam::Vec2& point) {}
} // namespace beam_calibration