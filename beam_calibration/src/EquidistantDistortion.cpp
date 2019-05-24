#include "beam_calibration/EquidistantDistortion.h"

namespace beam_calibration {

EquidistantDistortion::EquidistantDistortion(beam::VecX coeffs,
                                             DistortionType type)
    : DistortionModel(coeffs, type) {}

beam::Vec2 EquidistantDistortion::Distort(beam::Vec2& point) {
  beam::Vec2 coords;

  double x = point[0], y = point[1];
  beam::VecX coeffs = this->GetCoefficients();

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

beam::Vec2 EquidistantDistortion::Undistort(beam::Vec2& point) {}

cv::Mat EquidistantDistortion::UndistortImage(const cv::Mat& input_image,
                                              std::vector<double> intrinsics) {
  cv::Mat output_image;
  beam::VecX coeffs = this->GetCoefficients();
  std::vector<double> coefficients(
      coeffs.data(), coeffs.data() + coeffs.rows() * coeffs.cols());
  cv::InputArray K(intrinsics);
  cv::InputArray D(coefficients);
  cv::fisheye::undistortImage(input_image, output_image, K, D);
  return output_image;
}

} // namespace beam_calibration