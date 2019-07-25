// beam
#include "beam_cv/DepthMap.h"
#include "beam_cv/Morphology.h"
#include "beam_cv/RayCast.h"
#include "beam_utils/math.hpp"
// PCL
#include <pcl/io/pcd_io.h>
// std
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

namespace beam_cv {

DepthMap::DepthMap(std::shared_ptr<beam_calibration::CameraModel> model,
                   const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input) {
  this->SetCloud(cloud_input);
  this->SetModel(model);
}

void DepthMap::ExtractDepthMap(double threshold, int mask_size) {
  if (!point_cloud_initialized_ || !model_initialized_) {
    BEAM_CRITICAL("Variables not properly initialized.");
    throw std::runtime_error{"Variables not properly initialized."};
  }
  BEAM_INFO("Extracting Depth Image...");
  // create image mask where white pixels = projection hit
  cv::Mat hit_mask = this->CreateHitMask(mask_size);
  /// create image with 3 channels for coordinates
  depth_image_ = std::make_shared<cv::Mat>(model_->GetHeight(),
                                           model_->GetWidth(), CV_32FC1);
  // perform ray casting of cloud to create depth_image_
  beam_cv::RayCastXYZ(depth_image_, cloud_, hit_mask, threshold, model_,
                      HitBehaviour);

  depth_image_extracted_ = true;
  // compute min and max depth in the image
  min_depth_ = 1000, max_depth_ = 0;
  depth_image_->forEach<float>(
      [&](float& distance, const int* position) -> void {
        if (distance > max_depth_) { max_depth_ = distance; }
        if (distance < min_depth_) { min_depth_ = distance; }
      });
}

/*
 * TODO: parallelize the window searching
 */
void DepthMap::DepthInterpolation(int window_width, int window_height,
                                  float threshold, int iterations) {
  if (!this->CheckState()) {
    BEAM_CRITICAL("Variables not properly set.");
    throw std::runtime_error{"Variables not properly set."};
  }
  BEAM_INFO("Performing Depth Interpolation...");
  for (int iter = 0; iter < iterations; iter++) {
    cv::Mat dst = depth_image_->clone();
    depth_image_->forEach<float>([&](float& distance,
                                     const int* position) -> void {
      if (distance != 0.0) {
        int row = position[0], col = position[1];
        // each window contains: start_x, end_x, start_y, end_y
        std::vector<int> window_right{col, col + window_width,
                                      row - window_height, row + window_height};
        std::vector<int> window_left{col - window_width, col,
                                     row - window_height, row + window_height};
        std::vector<int> window_up{col - window_height, col + window_height,
                                   row - window_width, row};
        std::vector<int> window_down{col - window_height, col + window_height,
                                     row, row + window_width};
        std::vector<std::vector<int>> windows;
        windows.push_back(window_right);
        windows.push_back(window_left);
        windows.push_back(window_up);
        windows.push_back(window_down);
        float outside = iter * 1.0 + 2.0;
        for (std::vector<int> window : windows) {
          int start_x = window[0], end_x = window[1], start_y = window[2],
              end_y = window[3];
          beam::Vec2 point_f;
          float min_dist = 2 * (window_height * window_width);
          float found_depth = 0.0;
          bool found = false;
          if (start_y > 0 && start_x > 0 && end_y < depth_image_->rows &&
              end_x < depth_image_->cols) {
            for (int i = start_y; i < end_y; i++) {
              for (int j = start_x; j < end_x; j++) {
                float depth = depth_image_->at<float>(i, j);
                float dist_to_point =
                    sqrt((row - i) * (row - i) + (col - j) * (col - j));
                if (depth > 0 && abs(depth - distance) < threshold &&
                    dist_to_point < min_dist && dist_to_point > outside) {
                  min_dist = dist_to_point;
                  point_f[0] = i;
                  point_f[1] = j;
                  found = true;
                  found_depth = depth;
                }
              }
            }
            if (found) {
              int x = (point_f[0] + row) / 2;
              int y = (point_f[1] + col) / 2;
              float value = (distance + found_depth) / 2;
              dst.at<float>(x, y) = value;
            }
          }
        }
      }
    });
    *depth_image_ = dst;
  }
  BEAM_INFO("Depth Interpolation Completed");
}

void DepthMap::DepthCompletion(cv::Mat kernel) {
  if (!this->CheckState()) {
    BEAM_CRITICAL("Variables not properly set.");
    throw std::runtime_error{"Variables not properly set."};
  }
  BEAM_INFO("Performing Depth Completion...");
  // invert depth image
  cv::Mat image = depth_image_->clone();
  image.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance > 0.1) { distance = max_depth_ - distance; }
  });
  // dilate and close small holes
  cv::dilate(image, image, kernel);
  cv::morphologyEx(image, image, cv::MORPH_CLOSE, full_k_9);
  // dilate, then fill image with dilated
  cv::Mat dilated;
  cv::dilate(image, dilated, full_k_7);
  image.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance > 0.1) {
      distance = dilated.at<float>(position[0], position[1]);
    }
  });
  // close large holes
  cv::morphologyEx(image, image, cv::MORPH_CLOSE, full_k_15);
  // median blur to reduce noise
  cv::medianBlur(image, image, 5);
  cv::Mat dst = image.clone();
  // bilateral filter preserves structure
  cv::bilateralFilter(image, dst, 5, 0.5, 2.0);
  // invert back
  dst.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance > 0.1) { distance = max_depth_ - distance; }
  });
  *depth_image_ = dst;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DepthMap::ExtractPointCloud() {
  if (!this->CheckState()) {
    BEAM_CRITICAL("Variables not properly set.");
    throw std::runtime_error{"Variables not properly set."};
  }
  BEAM_INFO("Performing Point Cloud Construction...");
  pcl::PointCloud<pcl::PointXYZ>::Ptr dense_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (int row = 0; row < depth_image_->rows; row++) {
    for (int col = 0; col < depth_image_->cols; col++) {
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
  return dense_cloud;
}

/***********************Helper Functions**********************/

cv::Mat DepthMap::CreateHitMask(int mask_size) {
  // create image mask where white pixels = projection hit
  cv::Size image_s(model_->GetHeight(), model_->GetWidth());
  cv::Mat tmp = cv::Mat::zeros(image_s, CV_8UC3);
  cv::Mat hit_mask;
  for (uint32_t i = 0; i < cloud_->points.size(); i++) {
    beam::Vec3 point;
    point << cloud_->points[i].x, cloud_->points[i].y, cloud_->points[i].z;
    beam::Vec2 coords;
    coords = model_->ProjectPoint(point);
    uint16_t u = std::round(coords(0, 0)), v = std::round(coords(1, 0));
    if (u > 0 && v > 0 && v < model_->GetHeight() && u < model_->GetWidth()) {
      tmp.at<cv::Vec3b>(v, u).val[0] = 255;
    }
  }
  cv::dilate(tmp, hit_mask, cv::Mat(mask_size, mask_size, CV_8UC1),
             cv::Point(-1, -1), 1, 1, 1);
  return hit_mask;
}

cv::Mat DepthMap::VisualizeDepthImage() {
  if (!depth_image_extracted_) {
    BEAM_CRITICAL("Depth image not extracted.");
    throw std::runtime_error{"Depth image not extracted."};
  }
  cv::Mat gs_depth = cv::Mat(depth_image_->rows, depth_image_->cols, CV_8UC1);
  depth_image_->forEach<float>(
      [&](float& distance, const int* position) -> void {
        int scale = 255 / max_depth_;
        uint8_t pixel_value = (scale * distance);
        gs_depth.at<uchar>(position[0], position[1]) = pixel_value;
      });
  cv::applyColorMap(gs_depth, gs_depth, cv::COLORMAP_JET);
  return gs_depth;
}

bool DepthMap::CheckState() {
  bool state = false;
  if (point_cloud_initialized_ && depth_image_extracted_ &&
      model_initialized_) {
    state = true;
  }
  return state;
}

beam::Vec2 DepthMap::FindClosestNonZero(beam::Vec2 search_pixel) {
  beam::Vec2 found_pixel;
  float depth_l = 0.0, depth_r = 0.0, depth_u = 0.0, depth_d = 0.0;
  int x = search_pixel[0], y = search_pixel[1];
  int yr = y, yl = y, xu = x, xd = x;
  while (depth_l < 0.1 && depth_r < 0.1 && depth_u < 0.1 && depth_d < 0.1) {
    depth_u = depth_image_->at<float>(xu, y);
    xu--;
    depth_d = depth_image_->at<float>(xd, y);
    xd++;
    depth_l = depth_image_->at<float>(x, yl);
    yl--;
    depth_r = depth_image_->at<float>(x, yr);
    yr++;
    if (xu <= 0 || xd <= 0 || yl <= 0 || yr <= 0) { break; }
  }
  if (depth_l > 0.1) {
    found_pixel[0] = x;
    found_pixel[1] = yl;
  } else if (depth_r > 0.1) {
    found_pixel[0] = x;
    found_pixel[1] = yr;
  } else if (depth_u > 0.1) {
    found_pixel[0] = xu;
    found_pixel[1] = y;
  } else if (depth_d > 0.1) {
    found_pixel[0] = xd;
    found_pixel[1] = y;
  }
  return found_pixel;
}

beam::Vec3 DepthMap::GetXYZ(beam::Vec2 pixel) {
  float distance = depth_image_->at<float>(pixel[0], pixel[1]);
  if (distance == 0.0) {
    beam::Vec2 c = FindClosestNonZero(pixel);
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

std::shared_ptr<cv::Mat> DepthMap::GetDepthImage() {
  if (!depth_image_extracted_) {
    BEAM_CRITICAL("Depth image not extracted.");
    throw std::runtime_error{"Depth image not extracted."};
  }
  return depth_image_;
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
