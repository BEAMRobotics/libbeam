#include "beam_calibration/LadybugDistortion.h"

namespace beam_calibration {

LadybugDistortion::LadybugDistortion(unsigned int id) : cam_id_(id) {
  std::cout << "Created ladybug intrinsics object" << std::endl;
}

void LadybugDistortion::LoadConfig(std::string& file) {
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
}

beam::Vec2 LadybugDistortion::Distort(beam::Vec2& point) {
  beam::Vec2 pixel_out = {0, 0};
  lb_error_ = ladybugUnrectifyPixel(lb_context_, cam_id_, point[0], point[1],
                                    &pixel_out[0], &pixel_out[1]);
  LadybugCheckError();
  return pixel_out;
}

beam::Vec2 LadybugDistortion::Undistort(beam::Vec2& point) {
  beam::Vec2 pixel_out = {0, 0};
  lb_error_ = ladybugRectifyPixel(lb_context_, cam_id_, point[0], point[1],
                                  &pixel_out[0], &pixel_out[1]);
  LadybugCheckError();
  return pixel_out;
}

void LadybugDistortion::LadybugCheckError() {
  if (lb_error_ != LADYBUG_OK) {
    LOG_ERROR("Ladybug threw an error: %s", ladybugErrorToString(lb_error_));
  }
}
} // namespace beam_calibration