#include "beam_calibration/LadybugCamera.h"

namespace beam_calibration {

LadybugCamera::LadybugCamera(unsigned int id, std::string& file) : cam_id_(id) {
  lb_error_ = ladybugCreateContext(&lb_context_);
  LadybugCheckError();

  lb_error_ = ladybugLoadConfig(lb_context_, file.c_str());
  LadybugCheckError();

  lb_error_ =
      ladybugConfigureOutputImages(lb_context_, LADYBUG_ALL_RECTIFIED_IMAGES);
  LadybugCheckError();

  lb_error_ = ladybugSetOffScreenImageSize(
      lb_context_, LADYBUG_ALL_RECTIFIED_IMAGES, LB_FULL_HEIGHT, LB_FULL_WIDTH);
  LadybugCheckError();

  double focal_length = 0; // Focal length in pixels
  double cx = 0, cy = 0;   // Camera center in pixels
  lb_error_ =
      ladybugGetCameraUnitFocalLength(lb_context_, cam_id_, &focal_length);
  LadybugCheckError();

  lb_error_ = ladybugGetCameraUnitImageCenter(lb_context_, cam_id_, &cx, &cy);
  LadybugCheckError();

  // Set K matrix
  intrinsics_.resize(4);
  intrinsics_ << focal_length, focal_length, cx, cy;
}

beam::Vec2 LadybugCamera::ProjectPoint(beam::Vec3& point) {
  beam::Vec2 out_point;
  // Project point
  const double fx = intrinsics_[0], fy = intrinsics_[1], cx = intrinsics_[2],
               cy = intrinsics_[3];
  const double x = point[0], y = point[1], z = point[2];
  const double rz = 1.0 / z;
  out_point << (x * rz), (y * rz);
  // Distort point using distortion model
  out_point = this->Distort(out_point);
  // flip the coordinate system to be consistent with opencv convention shown:
  // http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/OWENS/LECT9/node2.html
  double xx = out_point[0], yy = out_point[1];
  out_point[0] = (fx * (-yy) + cx);
  out_point[1] = (fy * xx + cy);

  return out_point;
}

beam::Vec2 LadybugCamera::ProjectPoint(beam::Vec4& point) {
  beam::Vec3 new_point(point[0], point[1], point[2]);
  return ProjectPoint(new_point);
}

beam::Vec2 LadybugCamera::Undistort(beam::Vec2 pixel_in) {
  beam::Vec2 pixel_out = {0, 0};
  lb_error_ = ladybugRectifyPixel(lb_context_, cam_id_, pixel_in[0],
                                  pixel_in[1], &pixel_out[0], &pixel_out[1]);
  LadybugCheckError();
  return pixel_out;
}

beam::Vec2 LadybugCamera::Distort(beam::Vec2 pixel_in) {
  beam::Vec2 pixel_out = {0, 0};
  lb_error_ = ladybugUnrectifyPixel(lb_context_, cam_id_, pixel_in[0],
                                    pixel_in[1], &pixel_out[0], &pixel_out[1]);
  LadybugCheckError();
  return pixel_out;
}

void LadybugCamera::LadybugCheckError() {
  if (lb_error_ != LADYBUG_OK) {
    LOG_ERROR("Ladybug threw an error: %s", ladybugErrorToString(lb_error_));
  }
}

} // namespace beam_calibration