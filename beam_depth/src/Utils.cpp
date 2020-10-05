#include <beam_depth/Utils.h>

#include <algorithm>

#include <beam_utils/uf.hpp>

namespace beam_depth {

cv::Mat VisualizeDepthImage(const cv::Mat& input) {
  cv::Mat image = input.clone();
  float max_depth = 0;
  image.forEach<float>([&](float& distance, const int* position) -> void {
    (void)position;
    if (distance > max_depth) { max_depth = distance; }
  });

  int scale = 255 / max_depth;
  cv::Mat gs_depth;
  image.convertTo(gs_depth, CV_8UC1);

  image.forEach<float>([&](float& distance, const int* position) -> void {
    uint8_t pixel_value = (scale * distance);
    gs_depth.at<uchar>(position[0], position[1]) = pixel_value;
  });
  applyColorMap(gs_depth, gs_depth, cv::COLORMAP_JET);
  return gs_depth;
}

double PixelDistance(cv::Point2i p1, cv::Point2i p2) {
  double distance =
      sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
  return distance;
}

Eigen::Vector2i FindClosest(const Eigen::Vector2i& search_pixel,
                            const cv::Mat& depth_image) {
  cv::Point2i sp(search_pixel[0], search_pixel[1]);
  std::vector<double> distances;
  std::vector<cv::Point2i> pixels;
  depth_image.forEach<uchar>([&](uchar& pixel, const int* position) -> void {
    if (pixel > 0) {
      cv::Point2i p(position[0], position[1]);
      double d = PixelDistance(sp, p);
      distances.push_back(d);
      pixels.push_back(p);
    }
  });
  int min_index =
      std::min_element(distances.begin(), distances.end()) - distances.begin();
  Eigen::Vector2i output(pixels[min_index].x, pixels[min_index].y);
  return output;
}

std::vector<cv::Mat> SegmentMultiscale(const cv::Mat& depth_image) {
  float min_depth_ = 1000, max_depth_ = 0;
  depth_image.forEach<float>([&](float& distance, const int* position) -> void {
    (void)position;
    if (distance != 0.0) {
      if (distance > max_depth_) { max_depth_ = distance; }
      if (distance < min_depth_) { min_depth_ = distance; }
    }
  });

  float channel_width = (max_depth_ - min_depth_) / 4;
  cv::Mat empty =
      cv::Mat::zeros(cv::Size(depth_image.cols, depth_image.rows), CV_32FC1);
  cv::Mat channel1 = empty.clone(), channel2 = empty.clone(),
          channel3 = empty.clone(), channel4 = empty.clone();
  depth_image.forEach<float>([&](float& distance, const int* position) {
    if (distance >= min_depth_ && distance < min_depth_ + channel_width) {
      channel1.at<float>(position[0], position[1]) = distance;
    } else if (distance >= min_depth_ + channel_width &&
               distance < min_depth_ + 2 * channel_width) {
      channel2.at<float>(position[0], position[1]) = distance;
    } else if (distance >= min_depth_ + 2 * channel_width &&
               distance < min_depth_ + 3 * channel_width) {
      channel3.at<float>(position[0], position[1]) = distance;
    } else if (distance >= min_depth_ + 3 * channel_width &&
               distance < min_depth_ + 4 * channel_width) {
      channel4.at<float>(position[0], position[1]) = distance;
    }
  });
  std::vector<cv::Mat> segments = {channel1, channel2, channel3, channel4};
  return segments;
}

void SaveDepthImageBW(const cv::Mat& depth_image, const std::string& path) {
  cv::Mat depth_bw =
      cv::Mat::zeros(cv::Size(depth_image.cols, depth_image.rows), CV_8UC1);

  float min_depth = 1000, max_depth = 0;
  depth_image.forEach<float>([&](float& distance, const int* position) -> void {
    (void)position;
    if (distance != 0.0) {
      if (distance > max_depth) { max_depth = distance; }
      if (distance < min_depth) { min_depth = distance; }
    }
  });

  int scale = 255 / max_depth;

  depth_image.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance != 0) {
      uint8_t pixel_value = (scale * distance);
      depth_bw.at<uchar>(position[0], position[1]) = 255 - pixel_value;
    }
  });
  cv::imwrite(path, depth_bw);
}

} // namespace beam_depth
