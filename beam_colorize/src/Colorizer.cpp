#include <beam_utils/log.hpp>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <cv_bridge/cv_bridge.h>

#include "beam_colorize/Colorizer.h"
#include "beam_colorize/Projection.h"
#include "beam_colorize/RayTrace.h"

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
  input_point_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
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
  cv_bridge::CvImagePtr cv_img_ptr;
  cv_img_ptr =
      cv_bridge::toCvCopy(image_input, sensor_msgs::image_encodings::BGR8);
  image_ = std::make_shared<cv::Mat>(cv_img_ptr->image);
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

void Colorizer::CorrectImageGamma() {
  cv::Mat bgr_image = image_->clone();
  cv::Mat lab_image;
  cv::cvtColor(bgr_image, lab_image, CV_BGR2Lab);

  // Extract the L channel
  std::vector<cv::Mat> lab_planes(6);
  cv::split(lab_image, lab_planes); // now we have the L image in lab_planes[0]

  // apply the CLAHE algorithm to the L channel
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  // Explanation of ClipLimit
  // here:https://en.wikipedia.org/wiki/Adaptive_histogram_equalization#Contrast_Limited_AHE
  clahe->setClipLimit(3);
  cv::Mat dst;
  clahe->apply(lab_planes[0], dst);

  // Merge the the color planes back into an Lab image
  dst.copyTo(lab_planes[0]);
  cv::merge(lab_planes, lab_image);

  // convert back to RGB
  cv::cvtColor(lab_image, *image_, CV_Lab2BGR);
  cv::imwrite("/home/jake/adjusted.jpg", *image_);
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
