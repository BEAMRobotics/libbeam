#include "beam_cv/DepthMap.h"

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace beam_cv {

cv::Mat ExtractDepthMap(const cv::Mat& image,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        std::shared_ptr<beam_calibration::CameraModel> model) {
  /// create image with 3 channels for coordinates
  pcl::PointXYZ origin(0, 0, 0);
  cv::Mat depth_image = cv::Mat(image.rows, image.cols, CV_32FC1);
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);
  /// Compute depth image with point cloud
  depth_image.forEach<float>([&](float& pixel, const int* position) -> void {
    int v = position[0], u = position[1];
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
      if (distance < 0.02) {
        float point_dist = beam::distance(cloud->points[point_idx[0]], origin);
        pixel = point_dist;
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
  });

  return depth_image;
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

cv::Mat DepthCompletion(cv::Mat depth_image, cv::Mat kernel) {
  double max_depth = 0;
  depth_image.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance > max_depth) max_depth = distance;
  });
  depth_image.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance > 0.1) { distance = max_depth - distance; }
  });
  cv::dilate(depth_image, depth_image, kernel);
  cv::dilate(depth_image, depth_image, kernel);
  cv::morphologyEx(depth_image, depth_image, cv::MORPH_CLOSE,
                   beam::FULL_KERNEL_9);
  cv::Mat dilated = depth_image.clone();
  cv::dilate(dilated, depth_image, beam::FULL_KERNEL_7);
  depth_image.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance > 0.1) {
      distance = dilated.at<float>(position[0], position[1]);
    }
  });
  cv::morphologyEx(depth_image, depth_image, cv::MORPH_CLOSE,
                   beam::FULL_KERNEL_11);
  cv::medianBlur(depth_image, depth_image, 5);
  cv::Mat dst = depth_image.clone();
  cv::bilateralFilter(depth_image, dst, 5, 1.5, 2.0);

  dst.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance > 0.1) { distance = max_depth - distance; }
  });
  return dst;
}

} // namespace beam_cv