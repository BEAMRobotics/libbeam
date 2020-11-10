#include "beam_calibration/Ladybug.h"

namespace beam_calibration {

Ladybug::Ladybug(const std::string& file_path) {
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
  lb_error_ = ladybugGetCameraUnitImageCenter(lb_context_, cam_id_, &cx_, &cy_);
  LadybugCheckError();

  SetImageDims(LB_FULL_HEIGHT_, LB_FULL_WIDTH_);
  // Set intrinsics vector
  intrinsics_.resize(4);
  intrinsics_ << focal_length_, focal_length_, cy_, cx_;

}

opt<Eigen::Vector2d>
    Ladybug::ProjectPointPrecise(const Eigen::Vector3d& point, bool& outside_domain) {
  outside_domain = true;

  // check if point is behind image plane
  if (point[2] < 0) { 
    outside_domain = true;
    return {}; 
  }

  Eigen::Vector2d coords;
  Eigen::Vector3d x_proj, X_flip;
  Eigen::Matrix3d K;
  K << focal_length_, 0, cx_, 0, focal_length_, cy_, 0, 0, 1;
  // project
  x_proj = K * point;
  // normalize
  coords[0] = x_proj[0] / x_proj[2];
  coords[1] = x_proj[1] / x_proj[2];
  Eigen::Vector2d pixel_out = {0, 0};
  lb_error_ = ladybugUnrectifyPixel(lb_context_, cam_id_, coords[0], coords[1],
                                    &pixel_out[0], &pixel_out[1]);

  //unrectify function returns (-1,-1) if point projects outside of field of view
  if ((int)pixel_out[0] == -1 && (int)pixel_out[1] == -1)
    outside_domain = true; 
                                    
  if (PixelInImage(pixel_out)) { return pixel_out; }
  return {};
}

opt<Eigen::Vector2i> Ladybug::ProjectPoint(const Eigen::Vector3d& point, bool& outside_domain) {
  opt<Eigen::Vector2d> pixel = ProjectPointPrecise(point, outside_domain);
  if (pixel.has_value()) {
    Eigen::Vector2i pixel_rounded;
    pixel_rounded << std::round(pixel.value()[0]), std::round(pixel.value()[1]);
    return pixel_rounded;
  }
  return {};
}

opt<Eigen::Vector2i> Ladybug::ProjectPoint(const Eigen::Vector3d& point,
                                           Eigen::MatrixXd& J, bool& outside_domain) {
  BEAM_WARN(
      "Ladybug canot be re-calibrated, no effect on Jacobian matrix passed");
  return ProjectPoint(point, outside_domain);
}

opt<Eigen::Vector3d> Ladybug::BackProject(const Eigen::Vector2i& pixel) {
  Eigen::Vector2d pixel_out = {0, 0};
  Eigen::Vector3d out_point;
  lb_error_ = ladybugRectifyPixel(lb_context_, cam_id_, pixel[0], pixel[1],
                                  &pixel_out[0], &pixel_out[1]);
  out_point << (pixel_out[0] - cx_), (pixel_out[1] - cy_),
      (2 * focal_length_) / 2;
  out_point.normalize();
  LadybugCheckError();
  return out_point;
}

void Ladybug::SetCameraID(unsigned int id) {
  cam_id_ = id;

  lb_error_ =
      ladybugGetCameraUnitFocalLength(lb_context_, cam_id_, &focal_length_);
  LadybugCheckError();
  lb_error_ = ladybugGetCameraUnitImageCenter(lb_context_, cam_id_, &cx_, &cy_);
  LadybugCheckError();

  intrinsics_ << focal_length_, focal_length_, cy_, cx_;
}

void Ladybug::LadybugCheckError() {
  if (lb_error_ != LADYBUG_OK) {
    BEAM_CRITICAL("Ladybug threw an error: {}",
                  ladybugErrorToString(lb_error_));
  }
}

} // namespace beam_calibration
