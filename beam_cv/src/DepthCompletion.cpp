// beam
#include "beam_cv/DepthCompletion.h"
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

cv::Mat1f IPBasic(cv::Mat1f depth_img) {
  cv::Mat dst = depth_img.clone();
  // invert
  dst.forEach<float>([&](float& distance, const int* position) -> void {
    (void)position;
    if (distance > 0.1) { distance = 90.0 - distance; }
  });

  // dilate
  cv::Mat diamondKernel5 = beam::GetEllipseKernel(5);
  diamondKernel5.at<uchar>(1, 0) = 0;
  diamondKernel5.at<uchar>(1, 4) = 0;
  diamondKernel5.at<uchar>(3, 0) = 0;
  diamondKernel5.at<uchar>(3, 4) = 0;
  cv::dilate(dst, dst, diamondKernel5);

  // hole closing
  cv::morphologyEx(dst, dst, cv::MORPH_CLOSE, beam::GetFullKernel(5));
  // fill empty spaces with dilated values
  std::vector<cv::Point2i> empty_pixels;
  for (int row = 0; row < dst.rows; row++) {
    for (int col = 0; col < dst.cols; col++) {
      float distance = dst.at<float>(row, col);
      if (distance < 0.1) { empty_pixels.push_back(cv::Point2i(row, col)); }
    }
  }
  cv::Mat dilated;
  cv::dilate(dst, dilated, beam::GetFullKernel(7));
  for (int i = 0; i < empty_pixels.size(); i++) {
    dst.at<float>(empty_pixels[i].x, empty_pixels[i].y) =
        dilated.at<float>(empty_pixels[i].x, empty_pixels[i].y);
  }

  // median blur
  cv::medianBlur(dst, dst, 5);

  // bilateral filter
  cv::Mat copy = dst.clone();
  cv::bilateralFilter(copy, dst, 5, 1.5, 2.0);

  // invert
  dst.forEach<float>([&](float& distance, const int* position) -> void {
    (void)position;
    if (distance > 0.1) { distance = 90.0 - distance; }
  });

  return dst;
}

cv::Mat1f IDWInterpolation(cv::Mat1f depth_img, int window_size) {
  cv::Mat dst = depth_img.clone();
  cv::Mat copy = depth_img.clone();
  copy.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance == 0) {
      int row = position[0], col = position[1];
      // calculate window start and end points
      int start_row = row - (window_size / 2);
      int start_col = col - (window_size / 2);
      int end_row = row + (window_size / 2);
      int end_col = col + (window_size / 2);
      if (start_row < 0) start_row = 0;
      if (start_col < 0) start_col = 0;
      if (end_row > dst.rows) end_row = dst.rows;
      if (end_col > dst.cols) end_col = dst.cols;
      // fill vector with neighbourhood depth values and the distance to them
      std::vector<std::tuple<float, float>> closest_points;
      for (int i = start_row; i < end_row; i++) {
        for (int j = start_col; j < end_col; j++) {
          float depth = copy.at<float>(i, j);
          if (depth != 0) {
            float dist = ((i - row) * (i - row)) + ((j - col) * (j - col));
            dist = sqrt(dist);
            closest_points.push_back(std::make_tuple(dist, depth));
          }
        }
      }

      if (closest_points.size() >= 3) {
        float idw_numerator = 0.0;
        float idw_denominator = 0.0;
        for (int i = 0; i < closest_points.size(); i++) {
          std::tuple<float, float> pair = closest_points[i];
          float depth = std::get<1>(pair);
          float dist = std::get<0>(pair);
          idw_numerator += depth / dist;
          idw_denominator += 1 / dist;
        }
        float interpolated_depth = idw_numerator / idw_denominator;
        dst.at<float>(row, col) = interpolated_depth;
      }
    }
  });
  return dst;
}

cv::Mat1f MultiscaleInterpolation(cv::Mat1f depth_img) {
  // segment into 4 channels
  std::vector<cv::Mat> channels = beam_cv::SegmentMultiscale(depth_img);
  // perform completion on each
  for (int i = 0; i < 4; i++) {
    cv::Mat dst = channels[i];
    channels[i] = beam_cv::DepthInterpolation(21, 21, 5, channels[i]);
    channels[i] = beam_cv::DepthInterpolation(15, 15, 5, channels[i]);
    cv::Mat diamondKernel5 = beam::GetEllipseKernel(5);
    diamondKernel5.at<uchar>(1, 0) = 0;
    diamondKernel5.at<uchar>(1, 4) = 0;
    diamondKernel5.at<uchar>(3, 0) = 0;
    diamondKernel5.at<uchar>(3, 4) = 0;
    cv::dilate(channels[i], channels[i], diamondKernel5);
    cv::morphologyEx(channels[i], channels[i], cv::MORPH_CLOSE,
                     beam::GetFullKernel(5));
    cv::morphologyEx(channels[i], channels[i], cv::MORPH_CLOSE,
                     beam::GetEllipseKernel(11));
  }
  // recombine
  cv::Mat combined_depth =
      cv::Mat::zeros(cv::Size(depth_img.cols, depth_img.rows), CV_32F);
  for (int i = 0; i < 4; i++) {
    channels[i].forEach<float>(
        [&](float& distance, const int* position) -> void {
          if (distance != 0) {
            combined_depth.at<float>(position[0], position[1]) = distance;
          }
        });
  }
  float max_depth = 0;
  combined_depth.forEach<float>(
      [&](float& distance, const int* position) -> void {
        (void)position;
        if (distance != 0.0) {
          if (distance > max_depth) { max_depth = distance; }
        }
      });
  // invert
  cv::Mat dst = combined_depth.clone();
  dst.forEach<float>([&](float& distance, const int* position) -> void {
    (void)position;
    if (distance > 0.1) { distance = max_depth - distance; }
  });
  cv::morphologyEx(dst, dst, cv::MORPH_CLOSE, beam::GetEllipseKernel(21));
  cv::medianBlur(dst, dst, 5);
  // invert back
  dst.forEach<float>([&](float& distance, const int* position) -> void {
    (void)position;
    if (distance > 0.1) { distance = max_depth - distance; }
  });
  return dst;
}

} // namespace beam_cv