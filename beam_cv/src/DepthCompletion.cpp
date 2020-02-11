// beam
#include "beam_cv/DepthCompletion.h"
#include "beam_cv/DepthSuperpixels.h"
#include "beam_cv/Utils.h"

namespace beam_cv {
cv::Mat1f DepthInterpolation(int window_width, int window_height,
                             float threshold, cv::Mat1f depth_image) {
  cv::Mat dst = depth_image.clone();
  dst.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance != 0.0) {
      int row = position[0], col = position[1];
      // each window contains: start_x, end_x, start_y, end_y
      std::vector<int> window_right{col, col + window_width,
                                    row - window_height, row + window_height};
      std::vector<int> window_left{col - window_width, col, row - window_height,
                                   row + window_height};
      std::vector<int> window_up{col - window_height, col + window_height,
                                 row - window_width, row};
      std::vector<int> window_down{col - window_height, col + window_height,
                                   row, row + window_width};
      std::vector<std::vector<int>> windows;
      windows.push_back(window_right);
      windows.push_back(window_left);
      windows.push_back(window_up);
      windows.push_back(window_down);
      float outside = 3.0;
      // iterate through each window
      for (std::vector<int> window : windows) {
        int start_x = window[0], end_x = window[1], start_y = window[2],
            end_y = window[3];
        beam::Vec2 point_f;
        float min_dist = sqrt((window_width * window_width) +
                              (window_height * window_height));
        float found_depth = 0.0;
        bool found = false;
        if (start_y > 0 && start_x > 0 && end_y < dst.rows &&
            end_x < dst.cols) {
          // find closest point in window
          for (int i = start_y; i < end_y; i++) {
            for (int j = start_x; j < end_x; j++) {
              float depth = dst.at<float>(i, j);
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
            }
          }
        }
      }
    }
  });
  return dst;
}

cv::Mat1f KMeansCompletion(int K, cv::Mat rgb_image, cv::Mat1f depth_image) {
  cv::Mat di_copy = depth_image.clone();
  cv::Mat completed = cv::Mat::zeros(rgb_image.rows, rgb_image.cols, CV_32FC1);
  rgb_image = beam_cv::AdaptiveHistogram(rgb_image);
  rgb_image = beam_cv::KMeans(rgb_image, K);
  // find connected components
  std::map<int, std::vector<cv::Point2i>> sets =
      beam_cv::ConnectedComponents(rgb_image);
  // iterate over each component
  for (auto const& c : sets) {
    std::vector<cv::Point2i> points = c.second;
    std::vector<std::pair<float, cv::Point2i>> depth_points;
    // create vector of depth points that are within the component
    for (cv::Point2i p : points) {
      float dist = di_copy.at<float>(p.x, p.y);
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
          di_copy.at<float>(p.x, p.y) = 0.0;
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
        std::vector<std::pair<float, float>> closest_depths;
        for (auto const& d : depth_points) {
          float distance = beam_cv::PixelDistance(points[i], d.second);
          float depth = d.first;
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
  return completed;
}

cv::Mat1f SuperpixelCompletion(cv::Mat rgb_image, cv::Mat1f depth_image) {
  beam_cv::DepthSuperpixels DSP =
      beam_cv::DepthSuperpixels(rgb_image, depth_image, true);
  std::unordered_map<int, std::shared_ptr<beam_cv::SuperPixel>> superpixels =
      DSP.GetSuperpixels();

  for (auto& it : superpixels) {
    std::shared_ptr<beam_cv::SuperPixel> sp = it.second;
    for (cv::Point2i p : sp->pixels) {
      cv::Point2i p1, p2, p3, p4;
      float dp1 = 1000, dp2 = 1000, dp3 = 1000, dp4 = 1000;
      if (depth_image.at<float>(p.x, p.y) == 0) {
        for (std::tuple<cv::Point2i, float> depth_point : sp->depth_values_2) {
          cv::Point2i search_point = std::get<0>(depth_point);
          float dist = ((p.x - search_point.x) * (p.x - search_point.x)) +
                       ((p.y - search_point.y) * (p.y - search_point.y));
          dist = sqrt(dist);
          if (search_point.x < p.x && search_point.y < p.y && dist < dp1) {
            dp1 = dist;
            p1 = search_point;
          } else if (search_point.x < p.x && search_point.y >= p.y &&
                     dist < dp2) {
            dp2 = dist;
            p2 = search_point;
          } else if (search_point.x >= p.x && search_point.y >= p.y &&
                     dist < dp3) {
            dp3 = dist;
            p3 = search_point;
          } else if (search_point.x >= p.x && search_point.y < p.y &&
                     dist < dp4) {
            dp4 = dist;
            p4 = search_point;
          }
        }
      }
      if (dp1 != 1000 && dp2 != 1000 && dp3 != 1000 && dp4 != 1000) {
        float p1_f = depth_image.at<float>(p1.x, p1.y);
        float p2_f = depth_image.at<float>(p2.x, p2.y);
        float p3_f = depth_image.at<float>(p3.x, p3.y);
        float p4_f = depth_image.at<float>(p4.x, p4.y);
        depth_image.at<float>(p.x, p.y) = (p1_f + p2_f + p3_f + p4_f) / 4;
      } else if (dp1 < dp2 && dp1 < dp3 && dp1 < dp4) {
        depth_image.at<float>(p.x, p.y) = depth_image.at<float>(p1.x, p1.y);
      } else if (dp2 < dp1 && dp2 < dp3 && dp1 < dp4) {
        depth_image.at<float>(p.x, p.y) = depth_image.at<float>(p2.x, p2.y);
      } else if (dp3 < dp2 && dp3 < dp1 && dp1 < dp4) {
        depth_image.at<float>(p.x, p.y) = depth_image.at<float>(p3.x, p3.y);
      } else if (dp4 < dp2 && dp4 < dp1 && dp4 < dp3) {
        depth_image.at<float>(p.x, p.y) = depth_image.at<float>(p4.x, p4.y);
      }
    }
  }
  return depth_image;
}
} // namespace beam_cv