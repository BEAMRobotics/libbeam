#include "beam_calibration/FisheyeCamera.h"

namespace beam_calibration {

FisheyeCamera::FisheyeCamera() {
  type_ = CameraType::FISHEYE;
}

FisheyeCamera::FisheyeCamera(beam::VecX& intrinsics, beam::VecX& distortion,
                             uint32_t image_height, uint32_t image_width,
                             std::string frame_id, std::string date) {
  type_ = CameraType::FISHEYE;
  this->SetFrameID(frame_id);
  this->SetCalibrationDate(date);
  this->SetImageDims(image_height, image_width);
  this->SetIntrinsics(intrinsics);
  this->SetDistortionCoefficients(distortion);
}

beam::Vec2 FisheyeCamera::ProjectPoint(beam::Vec3& point) {
  beam::Vec2 out_point;
  if (intrinsics_valid_ && distortion_set_) {
    // Project point
    const double fx = this->GetFx(), fy = this->GetFy(), cx = this->GetCx(),
                 cy = this->GetCy();
    const double x = point[0], y = point[1], z = point[2];
    const double rz = 1.0 / z;
    out_point << (x * rz), (y * rz);
    // Distort point using distortion model
    out_point = this->DistortPoint(out_point);
    // flip the coordinate system to be consistent with opencv convention shown:
    // http:
    // homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/OWENS/LECT9/node2.html
    double xx = out_point[0], yy = out_point[1];
    out_point[0] = (fx * (-yy) + cx);
    out_point[1] = (fy * xx + cy);
  } else if (!intrinsics_valid_) {
    LOG_ERROR("Intrinsics not set, cannot project point.");
    throw std::invalid_argument{"Intrinsics nto set"};
  }

  return out_point;
}

beam::Vec2 FisheyeCamera::ProjectPoint(beam::Vec4& point) {
  bool homographic_form = (point[3] == 1);
  beam::Vec2 out_point;
  if (intrinsics_valid_ && homographic_form && distortion_set_) {
    beam::Vec3 new_point(point[0], point[1], point[2]);
    out_point = ProjectPoint(new_point);
  } else if (!intrinsics_valid_) {
    LOG_ERROR("Intrinsics not set, cannot project point.");
    throw std::invalid_argument{"Intrinsics nto set"};
  } else {
    LOG_ERROR("invalid entry, cannot project point: the point is not in "
              "homographic form, ");
    throw std::invalid_argument{"invalid point"};
  }
  return out_point;
}

beam::Vec2 FisheyeCamera::DistortPoint(beam::Vec2& point) {
  beam::Vec2 coords;

  double x = point[0], y = point[1];
  beam::VecX coeffs = this->GetDistortionCoefficients();

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

cv::Mat FisheyeCamera::UndistortImage(cv::Mat& input_image) {
  cv::Mat output_image;
  beam::Mat3 camera_matrix = this->GetCameraMatrix();
  // convert eigen to cv mat
  cv::Mat K(3, 3, CV_8UC1);
  cv::eigen2cv(camera_matrix, K);
  // convert eigen to cv mat
  beam::VecX distortion_coeffs = this->GetDistortionCoefficients();
  cv::Mat D(1, 4, CV_8UC1);
  cv::eigen2cv(distortion_coeffs, D);
  // undistort image
  cv::fisheye::undistortImage(input_image, output_image, K, D);
  return output_image;
}

} // namespace beam_calibration