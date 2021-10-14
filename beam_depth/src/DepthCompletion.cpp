#include "beam_depth/DepthCompletion.h"
#include "beam_depth/Utils.h"

namespace beam_depth {

void DepthInterpolation(int window_width, int window_height, float threshold,
                        cv::Mat& depth_image) {
  if (depth_image.type() != CV_32F) {
    BEAM_CRITICAL("Invalid OpenCV Mat type, requires CV_32F.");
    throw std::runtime_error{"Invalid OpenCV Mat type, requires CV_32F."};
  }
  depth_image.forEach<float>([&](float& distance, const int* position) -> void {
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
        Eigen::Vector2d point_f;
        float min_dist = sqrt((window_width * window_width) +
                              (window_height * window_height));
        float found_depth = 0.0;
        bool found = false;
        if (start_y > 0 && start_x > 0 && end_y < depth_image.rows &&
            end_x < depth_image.cols) {
          // find closest point in window
          for (int i = start_y; i < end_y; i++) {
            for (int j = start_x; j < end_x; j++) {
              float depth = depth_image.at<float>(i, j);
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
            if (depth_image.at<float>(x, y) == 0.0 &&
                abs(found_depth - distance) < threshold) {
              depth_image.at<float>(x, y) = value;
            }
          }
        }
      }
    }
  });
}

void IPBasic(cv::Mat& depth_image) {
  if (depth_image.type() != CV_32F) {
    BEAM_CRITICAL("Invalid OpenCV Mat type, requires CV_32F.");
    throw std::runtime_error{"Invalid OpenCV Mat type, requires CV_32F."};
  }
  // invert
  depth_image.forEach<float>([&](float& distance, const int* position) -> void {
    (void)position;
    if (distance > 0.1) { distance = 90.0 - distance; }
  });

  // dilate
  cv::Mat diamondKernel5 =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  diamondKernel5.at<uchar>(1, 0) = 0;
  diamondKernel5.at<uchar>(1, 4) = 0;
  diamondKernel5.at<uchar>(3, 0) = 0;
  diamondKernel5.at<uchar>(3, 4) = 0;
  cv::dilate(depth_image, depth_image, diamondKernel5);

  // hole closing
  cv::morphologyEx(depth_image, depth_image, cv::MORPH_CLOSE,
                   cv::Mat::ones(5, 5, CV_8U));
  // fill empty spaces with dilated values
  std::vector<cv::Point2i> empty_pixels;
  for (int row = 0; row < depth_image.rows; row++) {
    for (int col = 0; col < depth_image.cols; col++) {
      float distance = depth_image.at<float>(row, col);
      if (distance < 0.1) { empty_pixels.push_back(cv::Point2i(row, col)); }
    }
  }
  cv::Mat dilated;
  cv::dilate(depth_image, dilated, cv::Mat::ones(7, 7, CV_8U));
  for (uint32_t i = 0; i < empty_pixels.size(); i++) {
    depth_image.at<float>(empty_pixels[i].x, empty_pixels[i].y) =
        dilated.at<float>(empty_pixels[i].x, empty_pixels[i].y);
  }

  // median blur
  cv::medianBlur(depth_image, depth_image, 5);

  // bilateral filter
  cv::Mat copy = depth_image.clone();
  cv::bilateralFilter(copy, depth_image, 5, 1.5, 2.0);

  // invert
  depth_image.forEach<float>([&](float& distance, const int* position) -> void {
    (void)position;
    if (distance > 0.1) { distance = 90.0 - distance; }
  });
}

void IDWInterpolation(cv::Mat& depth_image, int window_size) {
  if (depth_image.type() != CV_32F) {
    BEAM_CRITICAL("Invalid OpenCV Mat type, requires CV_32F.");
    throw std::runtime_error{"Invalid OpenCV Mat type, requires CV_32F."};
  }
  cv::Mat copy = depth_image.clone();
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
      if (end_row > depth_image.rows) end_row = depth_image.rows;
      if (end_col > depth_image.cols) end_col = depth_image.cols;
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
        for (uint32_t i = 0; i < closest_points.size(); i++) {
          std::tuple<float, float> pair = closest_points[i];
          float depth = std::get<1>(pair);
          float dist = std::get<0>(pair);
          idw_numerator += depth / dist;
          idw_denominator += 1 / dist;
        }
        float interpolated_depth = idw_numerator / idw_denominator;
        depth_image.at<float>(row, col) = interpolated_depth;
      }
    }
  });
}

void MultiscaleInterpolation(cv::Mat& depth_image) {
  if (depth_image.type() != CV_32F) {
    BEAM_CRITICAL("Invalid OpenCV Mat type, requires CV_32F.");
    throw std::runtime_error{"Invalid OpenCV Mat type, requires CV_32F."};
  }
  // segment into 4 channels
  std::vector<cv::Mat> channels = beam_depth::SegmentMultiscale(depth_image);
  // perform completion on each
  for (int i = 0; i < 4; i++) {
    cv::Mat dst = channels[i];
    beam_depth::DepthInterpolation(21, 21, 5, channels[i]);
    beam_depth::DepthInterpolation(15, 15, 5, channels[i]);
    cv::Mat diamondKernel5 =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    diamondKernel5.at<uchar>(1, 0) = 0;
    diamondKernel5.at<uchar>(1, 4) = 0;
    diamondKernel5.at<uchar>(3, 0) = 0;
    diamondKernel5.at<uchar>(3, 4) = 0;
    cv::dilate(channels[i], channels[i], diamondKernel5);
    cv::morphologyEx(channels[i], channels[i], cv::MORPH_CLOSE,
                     cv::Mat::ones(5, 5, CV_8U));
    cv::morphologyEx(
        channels[i], channels[i], cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11)));
  }
  // recombine
  cv::Mat combined_depth =
      cv::Mat::zeros(cv::Size(depth_image.cols, depth_image.rows), CV_32F);
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
  depth_image = combined_depth.clone();
  depth_image.forEach<float>([&](float& distance, const int* position) -> void {
    (void)position;
    if (distance > 0.1) { distance = max_depth - distance; }
  });
  cv::morphologyEx(
      depth_image, depth_image, cv::MORPH_CLOSE,
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(21, 21)));
  cv::medianBlur(depth_image, depth_image, 5);
  // invert back
  depth_image.forEach<float>([&](float& distance, const int* position) -> void {
    (void)position;
    if (distance > 0.1) { distance = max_depth - distance; }
  });
}

} // namespace beam_depth