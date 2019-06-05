#include <pcl/kdtree/kdtree_flann.h>

#include "beam_colorize/RayTrace.h"

namespace beam_colorize {

RayTrace::RayTrace() : Colorizer() {}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RayTrace::ColorizePointCloud() const {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::copyPointCloud(*input_point_cloud_, *cloud_colored);

  if (!image_initialized_ || !point_cloud_initialized_ ||
      !intrinsics_initialized_ || input_point_cloud_->size() == 0) {
    throw std::runtime_error{"Colorizer not properly initialized."};
    return cloud_colored;
    LOG_ERROR("Colorizer not properly initialized.");
  }
  // remove points which will not be in the projection
  auto reduced_cloud =
      RayTrace::ReduceCloud(input_point_cloud_, image_, intrinsics_);
  auto input_cloud = std::get<0>(reduced_cloud);
  // indices stores a mapping back to the original cloud
  auto indices = std::get<1>(reduced_cloud);
  // create kdtree for faster searching
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud(input_cloud);
  // create image mask where white pixels = projection hit

  cv::Mat tmp = cv::Mat::zeros(image_->size(), CV_8UC3);

  cv::Mat hit_mask;
  for (uint32_t i = 0; i < input_cloud->points.size(); i++) {
    beam::Vec3 point;
    point << input_cloud->points[i].x, input_cloud->points[i].y,
        input_cloud->points[i].z;
    beam::Vec2 coords;

    coords = intrinsics_->ProjectPoint(point);

    uint16_t u = std::round(coords(0, 0)), v = std::round(coords(1, 0));
    if (u > 0 && v > 0 && v < image_->rows && u < image_->cols) {
      tmp.at<cv::Vec3b>(v, u).val[0] = 255;
    }
  }
  cv::dilate(tmp, hit_mask, cv::Mat(dilation_, dilation_, CV_8UC1),
             cv::Point(-1, -1), 1, 1, 1);

  // This lambda performs ray tracing in parallel on each pixel in the image
  uint32_t points_colored = 0;
  image_->forEach<Pixel>([&](Pixel& pixel, const int* position) -> void {
    // if the pixel actually traces to a point then continue
    cv::Vec3b colors = hit_mask.at<cv::Vec3b>(position[0], position[1]);
    if (colors.val[0] == 255) {
      // initialize ray
      beam::Vec3 ray(0, 0, 0);
      // get direction vector
      beam::Vec2 input_point(position[1], position[0]);
      beam::Vec3 point = intrinsics_->BackProject(input_point);
      // while loop to ray trace
      uint16_t raypt = 0;
      while (true) {
        // get point at end of ray
        pcl::PointXYZRGB search_point;
        search_point.x = ray(0, 0);
        search_point.y = ray(1, 0);
        search_point.z = ray(2, 0);
        /* Don't need thread lock - lookups on FLANN KDTree should be
         * thread-safe std::lock_guard<std::mutex> lock(mutex);
         */
        // search for closest point to ray
        std::vector<int> point_idx(1);
        std::vector<float> point_distance(1);
        kdtree.nearestKSearch(search_point, 1, point_idx, point_distance);
        float distance = sqrt(point_distance[0]);
        int idx = indices[point_idx[0]];
        // if the point is within 1cm then color it appropriately
        if (distance < hit_threshold_) {
          points_colored++;
          cloud_colored->points[idx].r = pixel.z;
          cloud_colored->points[idx].g = pixel.y;
          cloud_colored->points[idx].b = pixel.x;
          break;
        } else if (raypt >= max_ray_) {
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

  LOG_INFO("Coloured %d of %d total points.", (int)points_colored,
           (int)input_point_cloud_->points.size());
  return cloud_colored;
}

std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, std::vector<int>>
    RayTrace::ReduceCloud(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
        std::shared_ptr<cv::Mat> image,
        std::shared_ptr<beam_calibration::CameraModel> intrinsics) const {
  // cloud to search on
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<int> indices;
  beam::Vec3 point;
  for (uint32_t i = 0; i < input->points.size(); i++) {
    point << input->points[i].x, input->points[i].y, input->points[i].z;
    beam::Vec2 coords = intrinsics->ProjectPoint(point);
    uint16_t u = std::round(coords(0, 0)), v = std::round(coords(1, 0));
    uint16_t vmax = image->rows;
    uint16_t umax = image->cols;
    if (u > 0 && v > 0 && v < vmax && u < umax && point(2, 0) > 0) {
      pcl::PointXYZRGB new_point;
      new_point.x = point(0, 0);
      new_point.y = point(1, 0);
      new_point.z = point(2, 0);
      cloud->points.push_back(new_point);
      indices.push_back(i);
    }
  }
  return std::make_tuple(cloud, indices);
}

pcl::PointCloud<beam_containers::PointBridge>::Ptr
    RayTrace::ColorizeMask() const {
  return pcl::PointCloud<beam_containers::PointBridge>::Ptr();
}

} // namespace beam_colorize
