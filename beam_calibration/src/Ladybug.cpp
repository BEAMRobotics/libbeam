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

opt<Eigen::Vector2d>
    Ladybug::ProjectPointPrecise(const Eigen::Vector3d& point) {
  // check if point is behind image plane
  if (point[2] < 0) { return {}; }

  double x = point[0];
  double y = point[1];
  double z = point[2];

  // project
  Eigen::Vector2d point_projected;
  point_projected[0] = focal_length_ * x / z + cx_;
  point_projected[1] = focal_length_ * y / z + cy_;

  Eigen::Vector2d pixel_rectified(0, 0);
  lb_error_ = ladybugUnrectifyPixel(lb_context_, cam_id_, point_projected[0],
                                    point_projected[1], &pixel_rectified[0],
                                    &pixel_rectified[1]);

  if (PixelInImage(pixel_rectified)) { return pixel_rectified; }
  return {};
}

opt<Eigen::Vector2i> Ladybug::ProjectPoint(const Eigen::Vector3d& point) {
  opt<Eigen::Vector2d> pixel = ProjectPointPrecise(point);
  if (pixel.has_value()) {
    Eigen::Vector2i pixel_rounded;
    pixel_rounded << std::round(pixel.value()[0]), std::round(pixel.value()[1]);
    return pixel_rounded;
  }
  return {};
}

opt<Eigen::Vector2i> Ladybug::ProjectPoint(const Eigen::Vector3d& point,
                                           Eigen::MatrixXd& J) {
  BEAM_WARN(
      "Ladybug canot be re-calibrated, no effect on Jacobian matrix passed");
  return ProjectPoint(point);
}

opt<Eigen::Vector3d> Ladybug::BackProject(const Eigen::Vector2i& pixel) {
  Eigen::Vector2d pixel_out = {0, 0};
  Eigen::Vector3d out_point;
  lb_error_ = ladybugRectifyPixel(lb_context_, cam_id_, pixel[0], pixel[1],
                                  &pixel_out[0], &pixel_out[1]);
  out_point << (pixel_out[0] - cx_) / focal_length_,
      (pixel_out[1] - cy_) / focal_length_, 1;
  LadybugCheckError();
  return out_point;
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
