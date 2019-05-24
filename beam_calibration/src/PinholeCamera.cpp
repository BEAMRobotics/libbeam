#include "beam_calibration/PinholeCamera.h"

namespace beam_calibration {

PinholeCamera::PinholeCamera(CameraType camera_type, beam::VecX& intrinsics,
                             std::shared_ptr<DistortionModel> distortion,
                             uint32_t image_width, uint32_t image_height,
                             std::string frame_id, std::string date)
    : CameraModel(camera_type, intrinsics, distortion, image_width,
                  image_height, frame_id, date) {}

beam::Vec2 PinholeCamera::ProjectPoint(beam::Vec3& point) {
  beam::Vec2 out_point;
  // Project point
  const double fx = this->GetFx(), fy = this->GetFy(), cx = this->GetCx(),
               cy = this->GetCy();
  const double x = point[0], y = point[1], z = point[2];
  const double rz = 1.0 / z;
  out_point << (x * rz), (y * rz);
  // Distort point using distortion model
  out_point = this->GetDistortion()->Distort(out_point);
  // flip the coordinate system to be consistent with opencv convention shown:
  // http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/OWENS/LECT9/node2.html
  double xx = out_point[0], yy = out_point[1];
  out_point[0] = (fx * (-yy) + cx);
  out_point[1] = (fy * xx + cy);

  return out_point;
}

beam::Vec2 PinholeCamera::ProjectPoint(beam::Vec4& point) {
  beam::Vec3 new_point(point[0], point[1], point[2]);
  return ProjectPoint(new_point);
}

cv::Mat PinholeCamera::UndistortImage(cv::Mat& input_image) {
  beam::Mat3 camera_matrix = this->GetCameraMatrix();
  // convert eigen to cv mat
  cv::Mat K(3, 3, CV_8UC1);
  cv::eigen2cv(camera_matrix, K);
  // undistort image using appropriate model
  return this->GetDistortion()->UndistortImage(input_image, K);
}

} // namespace beam_calibration