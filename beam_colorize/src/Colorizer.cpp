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
}
void Colorizer::SetPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input) {
  cloud_in_lidar_frame_ = cloud_input;

  if (T_camera_lidar_.isIdentity()) {
    cloud_in_camera_frame_ = cloud_in_lidar_frame_;
  } else {
    pcl::transformPointCloud(*cloud_in_lidar_frame_, *cloud_in_camera_frame_,
                             Eigen::Affine3d(T_camera_lidar_));
  }
}

void Colorizer::SetPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input) {
  pcl::copyPointCloud(*cloud_input, *cloud_in_lidar_frame_);

  if (T_camera_lidar_.isIdentity()) {
    cloud_in_camera_frame_ = cloud_in_lidar_frame_;
  } else {
    pcl::transformPointCloud(*cloud_in_lidar_frame_, *cloud_in_camera_frame_,
                             Eigen::Affine3d(T_camera_lidar_));
  }
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

void Colorizer::SetTransform(const Eigen::Matrix4d& T_C_L) {
  T_camera_lidar_ = Eigen::Matrix4d::Identity();
  T_camera_lidar_ = T_C_L;

  // if point cloud already initialized, apply transformation now
  if (cloud_in_lidar_frame_->size() > 0) {
    pcl::transformPointCloud(*cloud_in_lidar_frame_, *cloud_in_camera_frame_,
                             Eigen::Affine3d(T_C_L));
  }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Colorizer::GetCloudInLidarFrame(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_colored) const {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored_in_lidar_frame =
      std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::transformPointCloud(*cloud_colored, *cloud_colored_in_lidar_frame,
                           Eigen::Affine3d(T_camera_lidar_).inverse());
  return cloud_colored_in_lidar_frame;
}

pcl::PointCloud<beam_containers::PointBridge>::Ptr
    Colorizer::GetCloudInLidarFrame(
        const pcl::PointCloud<beam_containers::PointBridge>::Ptr&
            cloud_pointbridge) const {
  pcl::PointCloud<beam_containers::PointBridge>::Ptr cloud_in_lidar_frame =
      std::make_shared<pcl::PointCloud<beam_containers::PointBridge>>();
  pcl::transformPointCloud(*cloud_pointbridge, *cloud_in_lidar_frame,
                           Eigen::Affine3d(T_camera_lidar_).inverse());
  return cloud_in_lidar_frame;
}

std::unique_ptr<Colorizer> Colorizer::Create(ColorizerType type) {
  if (type == ColorizerType::PROJECTION) {
    return std::make_unique<Projection>();
  } else if (type == ColorizerType::RAY_TRACE) {
    return std::make_unique<RayTrace>();
  } else {
    BEAM_ERROR("Colorizer type not yet implemented in factory method");
    return nullptr;
  }
}

} // namespace beam_colorize
