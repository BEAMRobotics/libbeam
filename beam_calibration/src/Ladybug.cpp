#include "beam_calibration/Ladybug.h"

namespace beam_calibration {

Ladybug::Ladybug(const std::string& file_path) {
  BEAM_INFO("Loading file: {}", file_path);

  type_ = CameraType::LADYBUG;
  lb_error_ = ladybugCreateContext(&lb_context_);
  LadybugCheckError();
  lb_error_ = ladybugLoadConfig(lb_context_, file_path.c_str());
  LadybugCheckError();
  lb_error_ =
      ladybugConfigureOutputImages(lb_context_, LADYBUG_ALL_RECTIFIED_IMAGES);
  LadybugCheckError();
  lb_error_ =
      ladybugSetOffScreenImageSize(lb_context_, LADYBUG_ALL_RECTIFIED_IMAGES,
                                   LB_FULL_HEIGHT_, LB_FULL_WIDTH_);
  LadybugCheckError();
  lb_error_ =
      ladybugGetCameraUnitFocalLength(lb_context_, cam_id_, &focal_length_);
  LadybugCheckError();
  lb_error_ = ladybugGetCameraUnitImageCenter(lb_context_, cam_id_, &cy_, &cx_);
  LadybugCheckError();

  SetImageDims(LB_FULL_HEIGHT_, LB_FULL_WIDTH_);
  // Set intrinsics vector
  intrinsics_.resize(4);
  intrinsics_ << focal_length_, focal_length_, cx_, cy_;
}

std::shared_ptr<CameraModel> Ladybug::Clone() {
  std::shared_ptr<Ladybug> clone = std::make_shared<Ladybug>();
  clone->type_ = CameraType::LADYBUG;
  clone->SetIntrinsics(this->GetIntrinsics());
  clone->SetImageDims(this->GetHeight(), this->GetWidth());
  clone->SetFrameID(this->GetFrameID());
  clone->SetCalibrationDate(this->GetCalibrationDate());
  return clone;
}

bool Ladybug::ProjectPoint(const Eigen::Vector3d& in_point,
                          Eigen::Vector2d& out_pixel, bool& in_image_plane,
                          std::shared_ptr<Eigen::MatrixXd> J) {
  double x = in_point[0];
  double y = in_point[1];
  double z = in_point[2];

  // project
  Eigen::Vector2d point_projected;
  point_projected[0] = focal_length_ * x / z + cx_;
  point_projected[1] = focal_length_ * y / z + cy_;

  Eigen::Vector2d pixel_rectified(0, 0);
  lb_error_ =
      ladybugUnrectifyPixel(lb_context_, cam_id_, point_projected[0],
                            point_projected[1], &out_pixel[0], &out_pixel[1]);

  // unrectify function returns (-1,-1) if point projects outside of field of
  // view
  if ((int)out_pixel[0] == -1 && (int)out_pixel[1] == -1) {
    in_image_plane = false;
    return false;
  } else {
    in_image_plane = false;
  }

  if (J) {
    BEAM_WARN("Ladybug cannot be recalibrated, no effect on Jacobian.");
  }

  return true;
}

bool Ladybug::BackProject(const Eigen::Vector2i& in_pixel,
                         Eigen::Vector3d& out_point) {
  Eigen::Vector2d pixel_out = {0, 0};
  lb_error_ = ladybugRectifyPixel(lb_context_, cam_id_, in_pixel[0],
                                  in_pixel[1], &pixel_out[0], &pixel_out[1]);
  out_point << (pixel_out[0] - cx_) / focal_length_,
      (pixel_out[1] - cy_) / focal_length_, 1;
  LadybugCheckError();
  return true;
}

void Ladybug::SetCameraID(unsigned int id) {
  cam_id_ = id;

  lb_error_ =
      ladybugGetCameraUnitFocalLength(lb_context_, cam_id_, &focal_length_);
  LadybugCheckError();
  lb_error_ = ladybugGetCameraUnitImageCenter(lb_context_, cam_id_, &cy_, &cx_);
  LadybugCheckError();

  intrinsics_ << focal_length_, focal_length_, cx_, cy_;
}

void Ladybug::LadybugCheckError() {
  if (lb_error_ != LADYBUG_OK) {
    BEAM_CRITICAL("Ladybug threw an error: {}",
                  ladybugErrorToString(lb_error_));
  }
}

} // namespace beam_calibration
