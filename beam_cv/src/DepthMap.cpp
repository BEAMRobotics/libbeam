#include "beam_cv/DepthMap.h"

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace beam_cv {
DepthMap::DepthMap(std::shared_ptr<beam_calibration::CameraModel> model,
                   const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input,
                   const cv::Mat& image_input) {
  cloud_ = cloud_input;
  point_cloud_initialized_ = true;
  image_ = std::make_shared<cv::Mat>(image_input);
  image_initialized_ = true;
  model_ = model;
  model_initialized_ = true;
}

void DepthMap::ExtractDepthMap(double threshold, int dilate, int mask_size) {
  if (!point_cloud_initialized_ || !model_initialized_) {
    BEAM_CRITICAL("Variables not properly initialized.");
    throw std::runtime_error{"Variables not properly initialized."};
  }
  // create image mask where white pixels = projection hit
  cv::Mat tmp = cv::Mat::zeros(image_->size(), CV_8UC3);
  cv::Mat hit_mask;
  for (uint32_t i = 0; i < cloud_->points.size(); i++) {
    beam::Vec3 point;
    point << cloud_->points[i].x, cloud_->points[i].y, cloud_->points[i].z;
    beam::Vec2 coords;
    coords = model_->ProjectPoint(point);
    uint16_t u = std::round(coords(0, 0)), v = std::round(coords(1, 0));
    if (u > 0 && v > 0 && v < image_->rows && u < image_->cols) {
      tmp.at<cv::Vec3b>(v, u).val[0] = 255;
    }
  }
  cv::dilate(tmp, hit_mask, cv::Mat(mask_size, mask_size, CV_8UC1),
             cv::Point(-1, -1), 1, 1, 1);
  /// create image with 3 channels for coordinates
  pcl::PointXYZ origin(0, 0, 0);
  depth_image_ =
      std::make_shared<cv::Mat>(image_->rows, image_->cols, CV_32FC1);
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud_);
  /// Compute depth image with point cloud
  depth_image_->forEach<float>([&](float& pixel, const int* position) -> void {
    int v = position[0], u = position[1];
    cv::Vec3b colors = hit_mask.at<cv::Vec3b>(v, u);
    if (colors.val[0] == 255) {
      beam::Vec3 ray(0, 0, 0);
      // get direction vector
      beam::Vec2 input_point(u, v);
      beam::Vec3 point = model_->BackProject(input_point);
      // while loop to ray trace
      uint16_t raypt = 0;
      while (true) {
        // get point at end of ray
        pcl::PointXYZ search_point;
        search_point.x = ray(0, 0);
        search_point.y = ray(1, 0);
        search_point.z = ray(2, 0);
        // search for closest point to ray
        std::vector<int> point_idx(1);
        std::vector<float> point_distance(1);
        kdtree.nearestKSearch(search_point, 1, point_idx, point_distance);
        float distance = sqrt(point_distance[0]);
        // if the point is within 1cm then color it appropriately
        if (distance < threshold) {
          pixel = beam::distance(cloud_->points[point_idx[0]], origin);
          break;
        } else if (raypt >= 20) {
          break;
        } else {
          raypt++;
          ray(0, 0) = ray(0, 0) + distance * point(0, 0);
          ray(1, 0) = ray(1, 0) + distance * point(1, 0);
          ray(2, 0) = ray(2, 0) + distance * point(2, 0);
        }
      }
    }
  });
  if (dilate > 0) { *depth_image_ = beam_cv::Dilate(*depth_image_, dilate); }
  depth_image_extracted_ = true;
}

void DepthMap::DepthInterpolation(int window_width, int window_height,
                                  float threshold, int dilate) {
  if (!depth_image_extracted_) {
    BEAM_CRITICAL("Depth image not extracted.");
    throw std::runtime_error{"Depth image not extracted."};
  }
  // O(nm) where n = number of pixels in image, m is window size
  cv::Mat dst = depth_image_->clone();
  depth_image_->forEach<float>(
      [&](float& distance, const int* position) -> void {
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
                    dist_to_point < min_dist && dist_to_point > 2.0) {
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
      });
  if (dilate > 0) { dst = beam_cv::Dilate(dst, dilate); }
  *depth_image_ = dst;
}

void DepthMap::DepthCompletion(cv::Mat kernel) {
  if (!depth_image_extracted_) {
    BEAM_CRITICAL("Depth image not extracted.");
    throw std::runtime_error{"Depth image not extracted."};
  }
  cv::Mat image = depth_image_->clone();
  double max_depth = 0;
  image.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance > max_depth) max_depth = distance;
  });
  image.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance > 0.1) { distance = max_depth - distance; }
  });
  cv::Mat full_k_9 = beam::GetFullKernel(9);
  cv::dilate(image, image, kernel);
  cv::dilate(image, image, kernel);
  cv::morphologyEx(image, image, cv::MORPH_CLOSE, full_k_9);
  cv::Mat dilated = beam_cv::Dilate(image, 7);
  image.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance > 0.1) {
      distance = dilated.at<float>(position[0], position[1]);
    }
  });
  cv::Mat full_k_17 = beam::GetFullKernel(17);
  cv::morphologyEx(image, image, cv::MORPH_CLOSE, full_k_17);
  cv::medianBlur(image, image, 5);
  cv::Mat dst = image.clone();
  cv::bilateralFilter(image, dst, 5, 1.5, 2.0);

  dst.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance > 0.1) { distance = max_depth - distance; }
  });
  *depth_image_ = dst;
}

cv::Mat DepthMap::VisualizeDepthImage() {
  if (!depth_image_extracted_) {
    BEAM_CRITICAL("Depth image not extracted.");
    throw std::runtime_error{"Depth image not extracted."};
  }
  cv::Mat gs_depth = cv::Mat(depth_image_->rows, depth_image_->cols, CV_8UC1);
  double max_distance = 0;
  depth_image_->forEach<float>(
      [&](float& distance, const int* position) -> void {
        if (distance > max_distance) max_distance = distance;
      });
  depth_image_->forEach<float>(
      [&](float& distance, const int* position) -> void {
        int scale = 255 / max_distance;
        uint8_t pixel_value = (scale * distance);
        gs_depth.at<uchar>(position[0], position[1]) = pixel_value;
      });
  cv::applyColorMap(gs_depth, gs_depth, cv::COLORMAP_JET);
  return gs_depth;
}

} // namespace beam_cv
