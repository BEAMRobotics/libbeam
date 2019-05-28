#include "beam_calibration/LadybugCamera.h"

namespace beam_calibration {

LadybugCamera::LadybugCamera(unsigned int id, std::string& file) : cam_id_(id) {
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
}

beam::Vec2 LadybugCamera::ProjectPoint(beam::Vec3& point) {
  beam::Vec2 out_point;
  if (intrinsics_valid_ && distortion_set_) {
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
    // Distort point using distortion model
    out_point = this->DistortPoint(coords);
  } else if (!intrinsics_valid_) {
    LOG_ERROR("Intrinsics not set, cannot project point.");
    throw std::invalid_argument{"Intrinsics nto set"};
  }

  return out_point;
}

beam::Vec2 LadybugCamera::ProjectPoint(beam::Vec4& point) {
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

beam::Vec2 LadybugCamera::DistortPoint(beam::Vec2& pixel_in) {
  beam::Vec2 pixel_out = {0, 0};
  lb_error_ = ladybugUnrectifyPixel(lb_context_, cam_id_, pixel_in[0],
                                    pixel_in[1], &pixel_out[0], &pixel_out[1]);
  LadybugCheckError();
  return pixel_out;
}

cv::Mat LadybugCamera::UndistortImage(cv::Mat& input_image) {
  return input_image;
}

void LadybugCamera::LadybugCheckError() {
  if (lb_error_ != LADYBUG_OK) {
    LOG_ERROR("Ladybug threw an error: %s", ladybugErrorToString(lb_error_));
  }
}

} // namespace beam_calibration