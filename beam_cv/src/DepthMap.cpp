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
  cv::dilate(*depth_image_, *depth_image_, beam::GetFullKernel(5));
  return num_extracted;
}

int DepthMap::DepthInterpolation(int window_width, int window_height,
                                 float threshold, int iterations) {
  if (!this->CheckState()) {
    BEAM_CRITICAL("Variables not properly set.");
    throw std::runtime_error{"Variables not properly set."};
  }
  int num_interpolated = 0;
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
        // iterate through each window
        for (std::vector<int> window : windows) {
          int start_x = window[0], end_x = window[1], start_y = window[2],
              end_y = window[3];
          beam::Vec2 point_f;
          float min_dist = sqrt((window_width * window_width) +
                                (window_height * window_height));
          float found_depth = 0.0;
          bool found = false;
          if (start_y > 0 && start_x > 0 && end_y < depth_image_->rows &&
              end_x < depth_image_->cols) {
            // find closest point in window
            for (int i = start_y; i < end_y; i++) {
              for (int j = start_x; j < end_x; j++) {
                float depth = depth_image_->at<float>(i, j);
                float dist_to_point =
                    sqrt((row - i) * (row - i) + (col - j) * (col - j));
                if (depth > 0 && dist_to_point < min_dist &&
                    dist_to_point > outside) {
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
              // input value if the found point is within threshold
              if (dst.at<float>(x, y) == 0.0 &&
                  abs(found_depth - distance) < threshold) {
                dst.at<float>(x, y) = value;
                ++num_interpolated;
              }
            }
          }
        }
      }
    });
    *depth_image_ = dst;
  }
  BEAM_INFO("Depth Interpolation Completed");
  return num_interpolated;
}

void DepthMap::KMeansCompletion(int K, cv::Mat img) {
  if (!this->CheckState()) {
    BEAM_CRITICAL("Variables not properly set.");
    throw std::runtime_error{"Variables not properly set."};
  }
  BEAM_INFO("K means completion...");
  cv::Mat di_copy = depth_image_->clone();
  cv::Mat completed = cv::Mat::zeros(img.rows, img.cols, CV_32FC1);
  img = beam_cv::AdaptiveHistogram(img);
  img = beam_cv::KMeans(img, K);
  // find connected components
  std::map<int, std::vector<cv::Point2i>> sets =
      beam_cv::ConnectedComponents(img);
  // iterate over each component
  for (auto const& c : sets) {
    std::vector<cv::Point2i> points = c.second;
    std::vector<std::pair<double, cv::Point2i>> depth_points;
    // create vector of depth points that are within the component
    for (cv::Point2i p : points) {
      double dist = di_copy.at<float>(p.x, p.y);
      if (dist > 0.0) { depth_points.push_back(std::make_pair(dist, p)); }
    }
    if (depth_points.size() >= 1) {
      // calculate mean of depth points
      float sum = 0;
      for (uint32_t n = 0; n < depth_points.size(); n++) {
        sum += depth_points[n].first;
      }
      float mean = sum / depth_points.size();
      // calculate standard deviation of depth_points
      float var = 0;
      for (uint32_t n = 0; n < depth_points.size(); n++) {
        var = var +
              ((depth_points[n].first - mean) * (depth_points[n].first - mean));
      }
      var /= depth_points.size();
      float sd = sqrt(var);
      // remove outliers
      for (uint32_t n = 0; n < depth_points.size(); n++) {
        float d = depth_points[n].first;
        cv::Point2i p = depth_points[n].second;
        if (d > 0.0 && (d > mean + 1 * (sd) || d < mean - 1 * (sd))) {
          depth_points.erase(depth_points.begin() + n);
          di_copy.at<double>(p.x, p.y) = 0.0;
          n--;
        }
      }
      // recalculate mean without outliers
      sum = 0;
      for (uint32_t n = 0; n < depth_points.size(); n++) {
        sum += depth_points[n].first;
      }
      mean = sum / depth_points.size();

      for (uint32_t i = 0; i < points.size(); i += 3) {
        // find closest depth points to pixel
        std::vector<std::pair<double, double>> closest_depths;
        for (auto const& d : depth_points) {
          double distance = beam_cv::PixelDistance(points[i], d.second);
          double depth = d.first;
          closest_depths.push_back(std::make_pair(distance, depth));
        }
        if (closest_depths.size() >= 5) {
          std::sort(closest_depths.begin(), closest_depths.end());
          closest_depths.resize(3);
          float d1 = closest_depths[0].first, d2 = closest_depths[1].first,
                d3 = closest_depths[2].first;
          float w1 = 1 / d1, w2 = 1 / d2, w3 = 1 / d3;
          float w_avg =
              (w1 * closest_depths[0].second + w2 * closest_depths[1].second +
               w3 * closest_depths[2].second) /
              (w1 + w2 + w3);
          completed.at<float>(points[i].x, points[i].y) = w_avg;
        }
      }
    }
  }
  cv::dilate(completed, completed, beam::GetEllipseKernel(11));
  cv::morphologyEx(completed, completed, cv::MORPH_CLOSE,
                   beam::GetFullKernel(11));
  *depth_image_ = completed;
  BEAM_INFO("Done.");
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

void DepthMap::SetDepthImage(cv::Mat1d input) {
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
