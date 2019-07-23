#include "beam_cv/DepthMap.h"

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace beam_cv {

cv::Mat ExtractDepthMap(const cv::Mat& image,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        std::shared_ptr<beam_calibration::CameraModel> model,
                        double threshold, int dilate) {
  // create image mask where white pixels = projection hit
  cv::Mat tmp = cv::Mat::zeros(image.size(), CV_8UC3);
  cv::Mat hit_mask;
  for (uint32_t i = 0; i < cloud->points.size(); i++) {
    beam::Vec3 point;
    point << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z;
    beam::Vec2 coords;
    coords = model->ProjectPoint(point);
    uint16_t u = std::round(coords(0, 0)), v = std::round(coords(1, 0));
    if (u > 0 && v > 0 && v < image.rows && u < image.cols) {
      tmp.at<cv::Vec3b>(v, u).val[0] = 255;
    }
  }
  cv::dilate(tmp, hit_mask, cv::Mat(31, 31, CV_8UC1), cv::Point(-1, -1), 1, 1,
             1);
  /// create image with 3 channels for coordinates
  pcl::PointXYZ origin(0, 0, 0);
  cv::Mat depth_image = cv::Mat(image.rows, image.cols, CV_32FC1);
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);
  /// Compute depth image with point cloud
  depth_image.forEach<float>([&](float& pixel, const int* position) -> void {
    int v = position[0], u = position[1];
    cv::Vec3b colors = hit_mask.at<cv::Vec3b>(v, u);
    if (colors.val[0] == 255) {
      beam::Vec3 ray(0, 0, 0);
      // get direction vector
      beam::Vec2 input_point(u, v);
      beam::Vec3 point = model->BackProject(input_point);
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
          pixel = beam::distance(cloud->points[point_idx[0]], origin);
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
  if (dilate > 0) { depth_image = beam_cv::Dilate(depth_image, dilate); }
  return depth_image;
}

cv::Mat DepthInterpolation(cv::Mat depth_image, int window_width,
                           int window_height, float threshold, int dilate) {
  // O(nm) where n = number of pixels in image, m is window size
  cv::Mat dst = depth_image.clone();
  depth_image.forEach<float>([&](float& distance, const int* position) -> void {
    int row = position[0], col = position[1];
    // each window contains: start_x, end_x, start_y, end_y
    std::vector<int> window_right{col, col + window_width, row - window_height,
                                  row + window_height};
    std::vector<int> window_left{col - window_width, col, row - window_height,
                                 row + window_height};
    std::vector<int> window_up{col - window_height, col + window_height,
                               row - window_width, row};
    std::vector<int> window_down{col - window_height, col + window_height, row,
                                 row + window_width};
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
      if (start_y > 0 && start_x > 0 && end_y < depth_image.rows &&
          end_x < depth_image.cols) {
        for (int i = start_y; i < end_y; i++) {
          for (int j = start_x; j < end_x; j++) {
            float depth = depth_image.at<float>(i, j);
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
  if (dilate > 1) { dst = beam_cv::Dilate(dst, dilate); }
  return dst;
}

cv::Mat DepthCompletion(cv::Mat depth_image, cv::Mat kernel) {
  double max_depth = 0;
  depth_image.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance > max_depth) max_depth = distance;
  });
  depth_image.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance > 0.1) { distance = max_depth - distance; }
  });
  cv::Mat full_k_9 = beam::GetFullKernel(9);
  cv::dilate(depth_image, depth_image, kernel);
  cv::dilate(depth_image, depth_image, kernel);
  cv::morphologyEx(depth_image, depth_image, cv::MORPH_CLOSE, full_k_9);
  cv::Mat dilated = beam_cv::Dilate(depth_image, 7);
  depth_image.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance > 0.1) {
      distance = dilated.at<float>(position[0], position[1]);
    }
  });
  cv::Mat full_k_11 = beam::GetFullKernel(11);
  cv::morphologyEx(depth_image, depth_image, cv::MORPH_CLOSE, full_k_11);
  cv::medianBlur(depth_image, depth_image, 5);
  cv::Mat dst = depth_image.clone();
  cv::bilateralFilter(depth_image, dst, 5, 1.5, 2.0);

  dst.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance > 0.1) { distance = max_depth - distance; }
  });
  return dst;
}

void DensifyPointCloud(cv::Mat depth_image,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                       std::shared_ptr<beam_calibration::CameraModel> model) {
  for (int i = 0; i < depth_image.rows; i++) {
    for (int j = 0; j < depth_image.cols; j++) {
      float distance = depth_image.at<float>(i, j);
      beam::Vec2 input_point(j, i);
      beam::Vec3 dir = model->BackProject(input_point);
      beam::Vec3 coords = distance * dir;
      pcl::PointXYZ cloud_point(coords[0], coords[1], coords[2]);
      cloud->points.push_back(cloud_point);
    }
  }
}

cv::Mat VisualizeDepthImage(cv::Mat depth_image) {
  cv::Mat gs_depth = cv::Mat(depth_image.rows, depth_image.cols, CV_8UC1);
  double max_distance = 0;
  depth_image.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance > max_distance) max_distance = distance;
  });
  depth_image.forEach<float>([&](float& distance, const int* position) -> void {
    int scale = 255 / max_distance;
    uint8_t pixel_value = (scale * distance);
    gs_depth.at<uchar>(position[0], position[1]) = pixel_value;
  });
  cv::applyColorMap(gs_depth, gs_depth, cv::COLORMAP_JET);
  return gs_depth;
}

} // namespace beam_cv

/* cv::Mat SegmentAndDilate(cv::Mat depth_image) {
  double max_depth = 0, min_depth = 1000;
  depth_image.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance != 0) {
      if (distance > max_depth) max_depth = distance;
      if (distance < min_depth) min_depth = distance;
    }
  });
  float channel_width = (max_depth - min_depth) / 4;
  cv::Mat channel1 = cv::Mat(depth_image.rows, depth_image.cols, CV_32FC1);
  cv::Mat channel2 = cv::Mat(depth_image.rows, depth_image.cols, CV_32FC1);
  cv::Mat channel3 = cv::Mat(depth_image.rows, depth_image.cols, CV_32FC1);
  cv::Mat channel4 = cv::Mat(depth_image.rows, depth_image.cols, CV_32FC1);

  depth_image.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance >= min_depth && distance < (min_depth + channel_width)) {
      channel1.at<float>(position[0], position[1]) = distance;
    } else if (distance >= min_depth + channel_width &&
               distance < min_depth + (2 * channel_width)) {
      channel2.at<float>(position[0], position[1]) = distance;
    } else if (distance >= min_depth + (2 * channel_width) &&
               distance < min_depth + (3 * channel_width)) {
      channel3.at<float>(position[0], position[1]) = distance;
    } else if (distance >= min_depth + (3 * channel_width) &&
               distance < min_depth + (4 * channel_width)) {
      channel4.at<float>(position[0], position[1]) = distance;
    }
  });
  cv::Mat dst = cv::Mat(depth_image.rows, depth_image.cols, CV_32FC1);
  cv::Mat kernel = beam::GetEllipseKernel(7);

  channel1 = beam_cv::DepthCompletion(channel1, kernel);
  channel2 = beam_cv::DepthCompletion(channel2, kernel);
  channel3 = beam_cv::DepthCompletion(channel3, kernel);
  channel4 = beam_cv::DepthCompletion(channel4, kernel);
  channel4.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance != 0) { dst.at<float>(position[0], position[1]) = distance; }
  });
  channel3.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance != 0) { dst.at<float>(position[0], position[1]) = distance; }
  });
  channel2.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance != 0) { dst.at<float>(position[0], position[1]) = distance; }
  });
  channel1.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance != 0) { dst.at<float>(position[0], position[1]) = distance; }
  });
  return dst;
}*/