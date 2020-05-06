#include "beam_calibration/include/Refactor/RadtanDistortion.h"

namespace beam_calibration {

RadtanDistortion::RadtanDistortion(beam::VecX coefficients) {
  distortion_coefficients_ = coefficients;
}

void RadtanDistortion::SetDistortionCoefficients(
    beam::VecX coefficients) const {
  distortion_coefficients_ = coefficients;
}

beam::Vec2 RadtanDistortion::DistortPixel(beam::Vec2 pixel) {
  beam::Vec2 coords;
  double x = pixel[0], y = pixel[1];

  double xx, yy,
      k1 = distortion_coefficients_[0], k2 = distortion_coefficients_[1],
      k3 = distortion_coefficients_[2], p1 = distortion_coefficients_[3],
      p2 = distortion_coefficients_[4];
  double mx2_u = x * x;
  double my2_u = y * y;
  double mxy_u = x * y;
  double rho2_u = mx2_u + my2_u;
  double rad_dist_u =
      k1 * rho2_u + k2 * rho2_u * rho2_u + k3 * rho2_u * rho2_u * rho2_u;
  xx = x + (x * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u));
  yy = y + (y * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u));

  coords << xx, yy;
  return coords;
}

beam::Vec2 RadtanDistortion::UndistortPixel(beam::Vec2 pixel) {
  constexpr int n = 200; // Max. number of iterations
  beam::Vec2 y = pixel;
  beam::Vec2 ybar = y;
  beam::Mat2 F;
  beam::Vec2 y_tmp;
  // Handle special case around image center.
  if (y.squaredNorm() < 1e-6) return y; // Point remains unchanged.
  for (int i = 0; i < n; ++i) {
    y_tmp = this->DistortPixel(distortion_coefficients_, ybar);
    F = this->ComputeJacobian(distortion_coefficients_, ybar);
    beam::Vec2 e(y - y_tmp);
    beam::Vec2 du = (F.transpose() * F).inverse() * F.transpose() * e;
    ybar += du;
  }
  y = ybar;
  return y;
}

beam::Mat2 RadtanDistortion::ComputeJacobian(beam::Vec2 pixel) {
  beam::Mat2 out_jacobian;
  double x = pixel[0];
  double y = pixel[1];
  double x2 = x * x;
  double y2 = y * y;
  double xy = x * y;
  const double& k1 = distortion_coefficients_[0];
  const double& k2 = distortion_coefficients_[1];
  const double& p1 = distortion_coefficients_[3];
  const double& p2 = distortion_coefficients_[4];
  double r2 = x2 + y2;
  double rad_dist_u = k1 * r2 + k2 * r2 * r2;
  const double duf_du = 1.0 + rad_dist_u + 2.0 * k1 * x2 + 4.0 * k2 * r2 * x2 +
                        2.0 * p1 * y + 6.0 * p2 * x;
  const double duf_dv =
      2.0 * k1 * xy + 4.0 * k2 * r2 * xy + 2.0 * p1 * x + 2.0 * p2 * y;
  const double dvf_du = duf_dv;
  const double dvf_dv = 1.0 + rad_dist_u + 2.0 * k1 * y2 + 4.0 * k2 * r2 * y2 +
                        2.0 * p2 * x + 6.0 * p1 * y;
  out_jacobian << duf_du, duf_dv, dvf_du, dvf_dv;
  return out_jacobian;
}

cv::Mat RadtanDistortion::UndistortImage(beam::Mat3 intrinsics,
                                         const cv::Mat& image) {
  uint32_t height = image->rows(), width = image->cols();
  cv::Mat output_image;
  cv::Mat K(3, 3, CV_32F);
  cv::eigen2cv(intrinsics, K);
  cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
  // convert eigen to cv mat
  beam::VecX new_coeffs(5);
  new_coeffs << distortion_coefficients_[0], distortion_coefficients_[1],
      distortion_coefficients_[3], distortion_coefficients_[4],
      distortion_coefficients_[2];
  cv::Mat D(1, 5, CV_8UC1);
  cv::eigen2cv(new_coeffs, D);
  // Undistort image
  cv::Mat map1, map2;
  cv::Size img_size = cv::Size(width, height);
  cv::initUndistortRectifyMap(K, D, R, K, img_size, CV_32FC1, map1, map2);
  cv::remap(image, output_image, map1, map2, 1);
  return output_image;
}

} // namespace beam_calibration