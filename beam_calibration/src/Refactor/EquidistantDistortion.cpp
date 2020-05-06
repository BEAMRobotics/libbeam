#include "beam_calibration/include/Refactor/EquidistantDistortion.h"

namespace beam_calibration {

EquidistantDistortion::EquidistantDistortion(beam::VecX coefficients) {
  distortion_coefficients_ = coefficients;
}

void EquidistantDistortion::SetDistortionCoefficients(
    beam::VecX coefficients) const {
  distortion_coefficients_ = coefficients;
}

beam::Vec2 EquidistantDistortion::DistortPixel(beam::Vec2 pixel) {
  beam::Vec2 coords;
  double x = point[0], y = point[1];

  const double k1 = distortion_coefficients_[0],
               k2 = distortion_coefficients_[1],
               k3 = distortion_coefficients_[2],
               k4 = distortion_coefficients_[3];

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

beam::Vec2 EquidistantDistortion::UndistortPixel(beam::Vec2 pixel) {
  constexpr int n = 30; // Max. number of iterations
  beam::Vec2 y = pixel;
  beam::Vec2 ybar = y;
  beam::Mat2 F;
  beam::Vec2 y_tmp;
  // Handle special case around image center.
  if (y.squaredNorm() < 1e-6) return y; // Point remains unchanged.
  for (int i = 0; i < n; ++i) {
    y_tmp = DistortPixel(distortion_coefficients_, ybar);
    F = ComputeJacobian(distortion_coefficients_, ybar);
    beam::Vec2 e(y - y_tmp);
    beam::Vec2 du = (F.transpose() * F).inverse() * F.transpose() * e;
    ybar += du;
  }
  y = ybar;
  return y;
}

beam::Mat2 EquidistantDistortion::ComputeJacobian(beam::Vec2 pixel) {
  beam::Mat2 out_jacobian;
  double x = pixel[0];
  double y = pixel[1];
  double x2 = x * x;
  double y2 = y * y;
  double r = sqrt(x2 + y2);
  const double& k1 = distortion_coefficients_[0];
  const double& k2 = distortion_coefficients_[1];
  const double& k3 = distortion_coefficients_[2];
  const double& k4 = distortion_coefficients_[3];
  // Handle special case around image center.
  if (r < 1e-10) { return out_jacobian; }
  double theta = atan(r);
  double theta2 = theta * theta;
  double theta4 = theta2 * theta2;
  double theta6 = theta2 * theta4;
  double theta8 = theta4 * theta4;
  double theta3 = theta2 * theta;
  double theta5 = theta4 * theta;
  double theta7 = theta6 * theta;
  const double duf_du =
      theta * 1.0 / r *
          (k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8 + 1.0) +
      x * theta * 1.0 / r *
          ((k2 * x * theta3 * 1.0 / r * 4.0) / (x2 + y2 + 1.0) +
           (k3 * x * theta5 * 1.0 / r * 6.0) / (x2 + y2 + 1.0) +
           (k4 * x * theta7 * 1.0 / r * 8.0) / (x2 + y2 + 1.0) +
           (k1 * x * theta * 1.0 / r * 2.0) / (x2 + y2 + 1.0)) +
      ((x2) * (k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8 + 1.0)) /
          ((x2 + y2) * (x2 + y2 + 1.0)) -
      (x2)*theta * 1.0 / pow(x2 + y2, 3.0 / 2.0) *
          (k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8 + 1.0);
  const double duf_dv =
      x * theta * 1.0 / r *
          ((k2 * y * theta3 * 1.0 / r * 4.0) / (x2 + y2 + 1.0) +
           (k3 * y * theta5 * 1.0 / r * 6.0) / (x2 + y2 + 1.0) +
           (k4 * y * theta7 * 1.0 / r * 8.0) / (x2 + y2 + 1.0) +
           (k1 * y * theta * 1.0 / r * 2.0) / (x2 + y2 + 1.0)) +
      (x * y * (k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8 + 1.0)) /
          ((x2 + y2) * (x2 + y2 + 1.0)) -
      x * y * theta * 1.0 / pow(x2 + y2, 3.0 / 2.0) *
          (k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8 + 1.0);
  const double dvf_du =
      y * theta * 1.0 / r *
          ((k2 * x * theta3 * 1.0 / r * 4.0) / (x2 + y2 + 1.0) +
           (k3 * x * theta5 * 1.0 / r * 6.0) / (x2 + y2 + 1.0) +
           (k4 * x * theta7 * 1.0 / r * 8.0) / (x2 + y2 + 1.0) +
           (k1 * x * theta * 1.0 / r * 2.0) / (x2 + y2 + 1.0)) +
      (x * y * (k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8 + 1.0)) /
          ((x2 + y2) * (x2 + y2 + 1.0)) -
      x * y * theta * 1.0 / pow(x2 + y2, 3.0 / 2.0) *
          (k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8 + 1.0);
  const double dvf_dv =
      theta * 1.0 / r *
          (k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8 + 1.0) +
      y * theta * 1.0 / r *
          ((k2 * y * theta3 * 1.0 / r * 4.0) / (x2 + y2 + 1.0) +
           (k3 * y * theta5 * 1.0 / r * 6.0) / (x2 + y2 + 1.0) +
           (k4 * y * theta7 * 1.0 / r * 8.0) / (x2 + y2 + 1.0) +
           (k1 * y * theta * 1.0 / r * 2.0) / (x2 + y2 + 1.0)) +
      ((y2) * (k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8 + 1.0)) /
          ((x2 + y2) * (x2 + y2 + 1.0)) -
      (y2)*theta * 1.0 / pow(x2 + y2, 3.0 / 2.0) *
          (k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8 + 1.0);

  out_jacobian << duf_du, duf_dv, dvf_du, dvf_dv;
  return out_jacobian;
}

cv::Mat EquidistantDistortion::UndistortImage(beam::Mat3 intrinsics,
                                              const cv::Mat& image) {
  uint32_t height = image->rows(), width = image->cols();
  cv::Mat output_image;
  cv::Mat K(3, 3, CV_32F);
  cv::eigen2cv(intrinsics, K);
  cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
  // convert eigen to cv mat
  cv::Mat D(1, 4, CV_8UC1);
  cv::eigen2cv(coeffs, D);
  // Undistort image
  cv::Mat map1, map2;
  cv::Size img_size = cv::Size(width, height);
  cv::Mat P;
  cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K, D, img_size, R, P);
  cv::fisheye::initUndistortRectifyMap(K, D, R, P, img_size, CV_32FC1, map1,
                                       map2);
  cv::remap(image, output_image, map1, map2, 1);
  return output_image;
}

} // namespace beam_calibration