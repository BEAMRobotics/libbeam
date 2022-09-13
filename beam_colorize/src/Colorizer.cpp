#include <beam_utils/log.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <beam_colorize/Colorizer.h>
#include <beam_colorize/Projection.h>
#include <beam_colorize/ProjectionOcclusionSafe.h>
#include <beam_colorize/RayTrace.h>
#include <beam_cv/OpenCVConversions.h>

namespace beam_colorize {
Colorizer::Colorizer() {
  image_distorted_ = true;
  image_initialized_ = false;
}
void Colorizer::SetPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in_camera_frame) {
  cloud_in_camera_frame_ = cloud_in_camera_frame;
}

void Colorizer::SetPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in_camera_frame) {
  pcl::copyPointCloud(*cloud_in_camera_frame, *cloud_in_camera_frame_);
}

void Colorizer::SetImage(const cv::Mat& image_input) {
  image_ = std::make_shared<cv::Mat>(image_input);
  image_initialized_ = true;
}

void Colorizer::SetImage(const sensor_msgs::Image& image_input) {
  image_ = std::make_shared<cv::Mat>(
      beam_cv::OpenCVConversions::RosImgToMat(image_input));
  image_initialized_ = true;
}

void Colorizer::SetIntrinsics(
    std::shared_ptr<beam_calibration::CameraModel> intrinsics) {
  camera_model_distorted_ = intrinsics;
  camera_model_undistorted_ = camera_model_distorted_->GetRectifiedModel();
  camera_model_ = camera_model_distorted_;
}

void Colorizer::SetDistortion(const bool& image_distored) {
  image_distorted_ = image_distored;
  if (image_distored) {
    camera_model_ = camera_model_distorted_;
  } else {
    camera_model_ = camera_model_undistorted_;
  }
}

std::unique_ptr<Colorizer> Colorizer::Create(ColorizerType type) {
  if (type == ColorizerType::PROJECTION) {
    return std::make_unique<Projection>();
  } else if (type == ColorizerType::PROJECTION_OCCLUSION_SAFE) {
    return std::make_unique<ProjectionOcclusionSafe>();
  } else if (type == ColorizerType::RAY_TRACE) {
    return std::make_unique<RayTrace>();
  } else {
    BEAM_ERROR("Colorizer type not yet implemented in factory method");
    return nullptr;
  }
}

} // namespace beam_colorize
