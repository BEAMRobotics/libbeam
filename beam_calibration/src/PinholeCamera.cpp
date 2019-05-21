#include "beam_calibration/PinholeCamera.h"

namespace beam_calibration {

PinholeCamera::PinholeCamera(beam_calibration::CameraType camera_type,
                             beam::VecX& intrinsics,
                             std::unique_ptr<DistortionModel> distortion,
                             uint32_t image_width, uint32_t image_height,
                             std::string frame_id, std::string date)
    : CameraModel(camera_type, intrinsics, std::move(distortion), image_width,
                  image_height, frame_id, date) {}

beam::Vec2 PinholeCamera::ProjectPoint(beam::Vec3& X) {}

beam::Vec2 PinholeCamera::ProjectPoint(beam::Vec4& X) {}
} // namespace beam_calibration