#include "beam_calibration/RadTanDistortion.h"

namespace beam_calibration {

RadTanDistortion::RadTanDistortion(beam::VecX coeffs) {
  SetCoefficients(coeffs);
}

beam::Vec2 RadTanDistortion::Distort(beam::Vec2& X) {}

beam::Vec2 RadTanDistortion::Undistort(beam::Vec2& X) {}
} // namespace beam_calibration