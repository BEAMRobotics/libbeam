#include <beam_utils/log.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include "beam_colorize/Colorizer.h"
#include "beam_colorize/Projection.h"
#include "beam_colorize/RayTrace.h"
#include <beam_cv/OpenCVConversions.h>

namespace beam_colorize {
Colorizer::Colorizer() {
  image_distorted_ = true;
  image_initialized_ = false;
  point_cloud_initialized_ = false;
  intrinsics_initialized_ = false;
  transform_set_ = false;
}
void Colorizer::SetPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input) {
  input_point_cloud_ = cloud_input;

  // if transform already set, transform cloud here
  if (transform_set_) {
    pcl::transformPointCloud(*input_point_cloud_, *input_point_cloud_, T_C_L_);
  }
  point_cloud_initialized_ = true;
}

void Colorizer::SetPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input) {
  input_point_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::copyPointCloud(*cloud_input, *input_point_cloud_);

  // if transform already set, transform cloud here
  if (transform_set_) {
    pcl::transformPointCloud(*input_point_cloud_, *input_point_cloud_, T_C_L_);
  }
  point_cloud_initialized_ = true;
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
  intrinsics_ = intrinsics;
  intrinsics_initialized_ = true;
}

void Colorizer::SetDistortion(const bool& image_distored) {
  image_distorted_ = image_distored;
}

void Colorizer::SetTransform(const Eigen::Affine3d& T_C_L) {
  T_C_L_ = T_C_L;

  // if point cloud already initialized, apply transfomration now
  if (point_cloud_initialized_) {
    pcl::transformPointCloud(*input_point_cloud_, *input_point_cloud_, T_C_L);
  }
  transform_set_ = true;
}

std::unique_ptr<Colorizer> Colorizer::Create(ColorizerType type) {
  if (type == ColorizerType::PROJECTION)
    return std::unique_ptr<Projection>(new Projection());
  else if (type == ColorizerType::RAY_TRACE)
    return std::unique_ptr<RayTrace>(new RayTrace());
  else
    return nullptr;
}

} // namespace beam_colorize
