#include "beam_calibration/LadybugCamera.h"

namespace beam_calibration {

LadybugCamera::LadybugCamera(std::string& file_location, uint cam_id)
    : cam_id_(cam_id) {
  BEAM_INFO("Loading file: {}", file);
  type_ = CameraType::LADYBUG;
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
  intrinsics_ << focal_length, focal_length, cy, cx;
  intrinsics_valid_ = true;
  this->SetImageDims(LB_FULL_HEIGHT, LB_FULL_WIDTH);
}

beam::Vec2 LadybugCamera::ProjectPoint(beam::Vec3 point) {
  beam::Vec2 out_point;

  // check if point is behind image plane
  if (point[2] < 0) { return Eigen::Vector2d(-1, -1); }

  if (intrinsics_valid_) {
    beam::Vec2 coords;
    beam::Vec3 x_proj, X_flip;
    beam::Mat3 K = this->GetCameraMatrix();
    // flip the coordinate system to be consistent with opencv convention shown
    // here:
    // http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/OWENS/LECT9/node2.html
    X_flip(0, 0) = -point(1, 0); // x = -y
    X_flip(1, 0) = point(0, 0);  // y = x
    X_flip(2, 0) = point(2, 0);  // z = z
    // project
    x_proj = K * X_flip;
    // normalize
    coords(0, 0) = x_proj(0, 0) / x_proj(2, 0);
    coords(1, 0) = x_proj(1, 0) / x_proj(2, 0);
    beam::Vec2 pixel_out = {0, 0};
    lb_error_ = ladybugUnrectifyPixel(lb_context_, cam_id_, coords[0],
                                      coords[1], &pixel_out[0], &pixel_out[1]);

  } else if (!intrinsics_valid_) {
    BEAM_CRITICAL("Intrinsics not set, cannot project point.");
    throw std::invalid_argument{"Intrinsics not set"};
  }
  return pixel_out;
}

beam::Vec3 LadybugCamera::BackProject(beam::Vec2 point) {
  beam::Vec2 pixel_in = point;
  beam::Vec2 pixel_out = {0, 0};
  beam::Vec3 out_point;
  lb_error_ = ladybugRectifyPixel(lb_context_, cam_id_, pixel_in[0],
                                  pixel_in[1], &pixel_out[0], &pixel_out[1]);
  out_point << (pixel_out[1] - this->GetCy()),
      (LB_FULL_WIDTH - this->GetCx() - pixel_out[0]),
      (this->GetFy() + this->GetFx()) / 2;
  out_point.normalize();
  LadybugCheckError();
  return out_point;
}

/**
 * TODO
 */
cv::Mat LadybugCamera::UndistortImage(cv::Mat input_image) {}

/**
 * TODO
 */
beam::Vec2 LadybugCamera::ProjectUndistortedPoint(beam::Vec3 point) {}

void LadybugCamera::LadybugCheckError() {
  if (lb_error_ != LADYBUG_OK) {
    LOG_ERROR("Ladybug threw an error: %s", ladybugErrorToString(lb_error_));
  }
}

} // namespace beam_calibration
