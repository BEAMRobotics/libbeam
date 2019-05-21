#include "beam_calibration/Ladybug.h"

namespace beam_calibration {

Ladybug::Ladybug(unsigned int id) : cam_id_(id) {
  std::cout << "Created ladybug intrinsics object" << std::endl;
}

void Ladybug::LoadJSON(std::string& file) {
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
  K_(0, 0) = focal_length;
  K_(0, 2) = cy;
  K_(1, 1) = focal_length;
  K_(1, 2) = cx;
  K_(2, 2) = 1;
  is_K_full_ = true;
  frame_id_ = "ladybug_cam" + std::to_string(cam_id_);
  std::cout << "Focal length: " << focal_length << std::endl;
  std::cout << "cx: " << cx << std::endl;
  std::cout << "cy: " << cy << std::endl;
}

std::string Ladybug::GetFrameId() {
  return frame_id_;
}

void Ladybug::SetFrameId(std::string& frame_id) {
  frame_id_ = frame_id;
}

beam::Vec2 Ladybug::GetImgDims() {
  return img_dims_;
}

void Ladybug::SetK(beam::Mat3 K) {}

beam::Mat3 Ladybug::GetK() {
  if (!is_K_full_) {
    LOG_ERROR("Intrinsics matrix empty.");
    throw std::invalid_argument{"no intrinsics matrix"};
  }
  return K_;
}

void Ladybug::SetImgDims(beam::Vec2& img_dims) {}

beam::Vec2 Ladybug::ProjectPoint(beam::Vec4& X) {
  return beam::Vec2();
}

beam::Vec2 Ladybug::ProjectPoint(beam::Vec3& X) {
  beam::Vec2 img_coords;
  if (is_K_full_) {
    img_coords = this->ApplyProjection(X);
  } else {
    LOG_ERROR("Intrinsics matrix empty, cannot project point.");
    throw std::invalid_argument{"no intrinsics matrix"};
  }
  return img_coords;
}

beam::Vec2 Ladybug::ProjectDistortedPoint(beam::Vec3& X) {
  beam::Vec2 img_coords;

  if (is_K_full_) {
    img_coords = this->ApplyDistortedProjection(X);
  } else {
    LOG_ERROR("Intrinsics matrix empty, cannot project point.");
    throw std::invalid_argument{"no intrinsics matrix"};
  }
  return img_coords;
}

beam::Vec2 Ladybug::ApplyDistortedProjection(beam::Vec3& X) {
  beam::Vec2 rectified_pixel = ProjectPoint(X);
  beam::Vec2 distorted_pixel = DistortPixel(rectified_pixel);

  return distorted_pixel;
}

beam::Vec2 Ladybug::UndistortPixel(beam::Vec2 pixel_in) {
  beam::Vec2 pixel_out = {0, 0};
  lb_error_ = ladybugRectifyPixel(lb_context_, cam_id_, pixel_in[0],
                                  pixel_in[1], &pixel_out[0], &pixel_out[1]);
  LadybugCheckError();
  //  std::cout << "Pixel in: [" << pixel_in[0] << ", " << pixel_in[1] << "] --
  //  Out : [" << pixel_out[0] << ", " << pixel_out[1] << "]." << std::endl;
  return pixel_out;
}

beam::Vec2 Ladybug::DistortPixel(beam::Vec2 pixel_in) {
  beam::Vec2 pixel_out = {0, 0};
  lb_error_ = ladybugUnrectifyPixel(lb_context_, cam_id_, pixel_in[0],
                                    pixel_in[1], &pixel_out[0], &pixel_out[1]);

  LadybugCheckError();
  return pixel_out;
}

bool Ladybug::PixelInImage(beam::Vec2 pixel_in) {
  if (pixel_in[0] < 0 || pixel_in[1] < 0 || pixel_in[0] > img_dims_[0] ||
      pixel_in[1] > img_dims_[1])
    return false;
  return true;
}

beam::Vec2 Ladybug::ApplyProjection(beam::Vec3& X) {
  beam::Vec2 coords;
  beam::Vec3 x_proj, X_flip;

  // flip the coordinate system to be consistent with opencv convention shown
  // here:
  // http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/OWENS/LECT9/node2.html
  X_flip(0, 0) = -X(1, 0); // x = -y
  X_flip(1, 0) = X(0, 0);  // y = x
  X_flip(2, 0) = X(2, 0);  // z = z

  // project
  x_proj = K_ * X_flip;

  // normalize
  coords(0, 0) = x_proj(0, 0) / x_proj(2, 0);
  coords(1, 0) = x_proj(1, 0) / x_proj(2, 0);
  return coords;
}

void Ladybug::ReadCameraCalibration(std::string calibration_path) {
  lb_error_ = ladybugCreateContext(&lb_context_);
}

void Ladybug::LadybugCheckError() {
  if (lb_error_ != LADYBUG_OK) {
    LOG_ERROR("Ladybug threw an error: %s", ladybugErrorToString(lb_error_));
  }
}

std::ostream& operator<<(std::ostream& out, Ladybug& ladybug) {
  out << "Frame ID: " << ladybug.GetFrameId() << std::endl;
  out << "Image center: [cx, cy] = [" << ladybug.GetK()(0, 2) << ", "
      << ladybug.GetK()(1, 2) << "]." << std::endl;
  out << "Focal length: [fx, fy] = [" << ladybug.GetK()(0, 0) << ", "
      << ladybug.GetK()(1, 1) << "]." << std::endl;
  out << "Image dimensions: [w, h] = [" << ladybug.GetImgDims()[0] << ", "
      << ladybug.GetImgDims()[1] << "]." << std::endl;
  return out;
}

}