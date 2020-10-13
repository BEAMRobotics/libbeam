#include "beam_utils/angles.hpp"

namespace beam {

double WrapToPi(double angle) {
  double wrapped_angle = WrapToTwoPi(angle + M_PI) - M_PI;
  return wrapped_angle;
}

double WrapToTwoPi(double angle) {
  double wrapped_angle = fmod(angle, 2 * M_PI);
  if (wrapped_angle < 0) { wrapped_angle += 2 * M_PI; }
  return wrapped_angle;
}

double Deg2Rad(double d) {
  return d * (M_PI / 180);
}

double Rad2Deg(double r) {
  return r * (180 / M_PI);
}

double WrapTo180(double euler_angle) {
  return Rad2Deg(WrapToPi(Deg2Rad(euler_angle)));
}

double WrapTo360(double euler_angle) {
  return Rad2Deg(WrapToTwoPi(Deg2Rad(euler_angle)));
}

double GetSmallestAngleErrorDeg(double angle1, double angle2) {
  return Rad2Deg(GetSmallestAngleErrorRad(Deg2Rad(angle1), Deg2Rad(angle2)));
}

double GetSmallestAngleErrorRad(double angle1, double angle2) {
  double angle1_wrapped = WrapToTwoPi(angle1);
  double angle2_wrapped = WrapToTwoPi(angle2);
  if (std::min(angle1_wrapped, angle2_wrapped) < M_PI / 2 &&
      std::max(angle1_wrapped, angle2_wrapped) > 3 * M_PI / 2) {
    return 2 * M_PI - (std::max(angle1_wrapped, angle2_wrapped) -
                       std::min(angle1_wrapped, angle2_wrapped));
  } else {
    return std::max(angle1_wrapped, angle2_wrapped) -
           std::min(angle1_wrapped, angle2_wrapped);
  }
}
} // namespace beam
