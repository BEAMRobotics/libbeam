// beam
#include "beam_cv/DepthMap.h"
#include "beam_cv/RayCast.h"
#include "beam_cv/Utils.h"
#include "beam_utils/math.hpp"
// PCL
#include <pcl/io/pcd_io.h>

namespace beam_cv {

DepthMap::DepthMap(std::shared_ptr<beam_calibration::CameraModel> model,
                   const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input) {
  this->SetCloud(cloud_input);
  this->SetModel(model);
}

int DepthMap::ExtractDepthMap(double threshold, int mask_size) {
  if (!point_cloud_initialized_ || !model_initialized_) {
    BEAM_CRITICAL("Variables not properly initialized.");
    throw std::runtime_error{"Variables not properly initialized."};
  }
  BEAM_INFO("Extracting Depth Image...");
  // create image mask where white pixels = projection hit
  cv::Mat hit_mask = beam_cv::CreateHitMask(mask_size, model_, cloud_);
  /// create image with 3 channels for coordinates
  depth_image_ = std::make_shared<cv::Mat>(model_->GetHeight(),
                                           model_->GetWidth(), CV_32FC1);
  // perform ray casting of cloud to create depth_image_
  beam_cv::RayCastXYZ(depth_image_, cloud_, hit_mask, threshold, model_,
                      HitBehaviour);

  depth_image_extracted_ = true;
  int num_extracted = 0;
  // compute min and max depth in the image
  min_depth_ = 1000, max_depth_ = 0;
  depth_image_->forEach<float>(
      [&](float& distance, const int* position) -> void {
        (void)position;
        if (distance != 0.0) {
          num_extracted++;
          if (distance > max_depth_) { max_depth_ = distance; }
          if (distance < min_depth_) { min_depth_ = distance; }
        }
      });
  // cv::dilate(*depth_image_, *depth_image_, beam::GetFullKernel(5));
  return num_extracted;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DepthMap::ExtractPointCloud() {
  if (!this->CheckState()) {
    BEAM_CRITICAL("Variables not properly set.");
    throw std::runtime_error{"Variables not properly set."};
  }
  BEAM_INFO("Performing Point Cloud Construction...");
  pcl::PointCloud<pcl::PointXYZ>::Ptr dense_cloud =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (int row = 0; row < depth_image_->rows; row += 2) {
    for (int col = 0; col < depth_image_->cols; col += 2) {
      float distance = depth_image_->at<float>(row, col);
      if (distance > 0) {
        beam::Vec2 pixel(col, row);
        beam::Vec3 direction = model_->BackProject(pixel);
        beam::Vec3 coords = distance * direction;
        pcl::PointXYZ point(coords[0], coords[1], coords[2]);
        dense_cloud->points.push_back(point);
      }
    }
  }
  dense_cloud->width = 1;
  dense_cloud->height = dense_cloud->points.size();
  return dense_cloud;
}

/***********************Helper Functions**********************/

bool DepthMap::CheckState() {
  bool state = false;
  if (point_cloud_initialized_ && depth_image_extracted_ &&
      model_initialized_) {
    state = true;
  }
  return state;
}

beam::Vec3 DepthMap::GetXYZ(beam::Vec2 pixel) {
  float distance = depth_image_->at<float>(pixel[0], pixel[1]);
  if (distance == 0.0) {
    beam::Vec2 c = beam_cv::FindClosest(pixel, *depth_image_);
    distance = depth_image_->at<float>(c[0], c[1]);
  }
  beam::Vec3 direction = model_->BackProject(pixel);
  beam::Vec3 coords = distance * direction;
  return coords;
}

float DepthMap::GetDistance(beam::Vec2 p1, beam::Vec2 p2) {
  beam::Vec3 coord1 = GetXYZ(p1), coord2 = GetXYZ(p2);
  return beam::distance(coord1, coord2);
}

/***********************Getters/Setters**********************/

pcl::PointCloud<pcl::PointXYZ>::Ptr DepthMap::GetCloud() {
  if (!point_cloud_initialized_) {
    BEAM_CRITICAL("Cloud not set.");
    throw std::runtime_error{"Cloud not set."};
  }
  return cloud_;
}

void DepthMap::SetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
  cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::copyPointCloud(*input_cloud, *cloud_);
  point_cloud_initialized_ = true;
}

cv::Mat DepthMap::GetDepthImage() {
  if (!depth_image_extracted_) {
    BEAM_CRITICAL("Depth image not extracted.");
    throw std::runtime_error{"Depth image not extracted."};
  }
  return *depth_image_;
}

void DepthMap::SetDepthImage(cv::Mat input) {
  depth_image_ = std::make_shared<cv::Mat>(input);
  depth_image_extracted_ = true;
  min_depth_ = 1000, max_depth_ = 0;
  depth_image_->forEach<float>(
      [&](float& distance, const int* position) -> void {
        (void)position;
        if (distance != 0.0) {
          if (distance > max_depth_) { max_depth_ = distance; }
          if (distance < min_depth_) { min_depth_ = distance; }
        }
      });
}

std::shared_ptr<beam_calibration::CameraModel> DepthMap::GetModel() {
  if (!model_initialized_) {
    BEAM_CRITICAL("Camera model not set.");
    throw std::runtime_error{"Camera model not set."};
  }
  return model_;
}

void DepthMap::SetModel(
    std::shared_ptr<beam_calibration::CameraModel> input_model) {
  model_ = input_model;
  model_initialized_ = true;
}

void HitBehaviour(std::shared_ptr<cv::Mat> image,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                  const int* position, int index) {
  pcl::PointXYZ origin(0, 0, 0);
  image->at<float>(position[0], position[1]) =
      beam::distance(cloud->points[index], origin);
}

} // namespace beam_cv
