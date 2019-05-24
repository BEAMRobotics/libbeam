#include "beam_calibration/RadTanDistortion.h"

namespace beam_calibration {

RadTanDistortion::RadTanDistortion(beam::VecX coeffs, DistortionType type)
    : DistortionModel(coeffs, type) {}

beam::Vec2 RadTanDistortion::Distort(beam::Vec2& point) {
  beam::Vec2 coords;
  double x = point[0], y = point[1];
  beam::VecX coeffs = this->GetCoefficients();

  double xx, yy, r2, k1 = coeffs[0], k2 = coeffs[1], k3 = coeffs[2],
                     p1 = coeffs[3], p2 = coeffs[4];
  r2 = x * x + y * y;
  double quotient = (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
  xx = x * quotient + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
  yy = y * quotient + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;

  coords << xx, yy;
  return coords;
}

beam::Vec2 RadTanDistortion::Undistort(beam::Vec2& point) {
  return point;
}

cv::Mat RadTanDistortion::UndistortImage(const cv::Mat& input_image,
                                         cv::Mat K) {
  cv::Mat output_image;
  // convert eigen to cv mat
  beam::VecX distortion_coeffs = this->GetCoefficients();
  cv::Mat D(1, 5, CV_8UC1);
  cv::eigen2cv(distortion_coeffs, D);
  // undistort image
  cv::undistort(input_image, output_image, K, D);
  return output_image;
}

} // namespace beam_calibration