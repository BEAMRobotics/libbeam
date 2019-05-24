#include "beam_calibration/PinholeCamera.h"

namespace beam_calibration {

PinholeCamera::PinholeCamera(beam::VecX& intrinsics,
                             std::shared_ptr<DistortionModel> distortion,
                             uint32_t image_width, uint32_t image_height,
                             std::string frame_id, std::string date) {
  type_ = CameraType::PINHOLE;
  this->SetFrameID(frame_id);
  this->SetCalibrationDate(date);
  this->SetImageDims(image_height, image_width);
  this->SetIntrinsics(intrinsics);
  this->SetDistortion(std::move(distortion));
}

beam::Vec2 PinholeCamera::ProjectPoint(beam::Vec3& point) {
  beam::Vec2 out_point;
  if (intrinsics_valid_ && distortion_set_) {
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
  } else if (!intrinsics_valid_) {
    LOG_ERROR("Intrinsics not set, cannot project point.");
    throw std::invalid_argument{"Intrinsics nto set"};
  }

  return out_point;
}

beam::Vec2 PinholeCamera::ProjectPoint(beam::Vec4& point) {
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

cv::Mat PinholeCamera::UndistortImage(cv::Mat& input_image) {
  beam::Mat3 camera_matrix = this->GetCameraMatrix();
  // convert eigen to cv mat
  cv::Mat K(3, 3, CV_8UC1);
  cv::eigen2cv(camera_matrix, K);
  // undistort image using appropriate model
  return this->GetDistortion()->UndistortImage(input_image, K);
}

} // namespace beam_calibration