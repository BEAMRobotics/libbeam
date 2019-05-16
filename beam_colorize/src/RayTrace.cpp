#include "beam_colorize/RayTrace.h"

namespace beam_colorize {

RayTrace::RayTrace() {
  image_distored_ = true;
  image_initialized_ = false;
  point_cloud_initialized_ = false;
  intrinsics_initialized_ = false;
  transform_set_ = false;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    RayTrace::ColorizePointCloud(int dilation) const {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::copyPointCloud(*input_point_cloud_, *cloud_colored);

  if (!image_initialized_ || !point_cloud_initialized_ ||
      !intrinsics_initialized_) {
    return cloud_colored;
    throw std::runtime_error{"Colorizer not properly initialized."};
    LOG_ERROR("Colorizer not properly initialized.");
  }

  // store intrinsics of camera
  beam::Mat3 K = intrinsics_->GetK();
  double f = (K(0, 0) + K(1, 1)) / 2, cx = K(0, 2), cy = K(1, 2);
  // remove points which will not be in the projection
  auto reduced_cloud =
      RayTrace::ReduceCloud_(input_point_cloud_, image_, intrinsics_);
  auto input_cloud = std::get<0>(reduced_cloud);
  // indices stores a mapping back to the original cloud
  auto indices = std::get<1>(reduced_cloud);
  // create kdtree for faster searching
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud(input_cloud);

  // create image mask where white pixels = projection hit
  cv::Mat tmp(image_->rows, image_->cols, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat hit_mask;
  if (dilation == -1) {
    cv::bitwise_not(tmp, hit_mask);
  } else {
    for (uint32_t i = 0; i < input_cloud->points.size(); i++) {
      beam::Vec3 point;
      point << input_cloud->points[i].x, input_cloud->points[i].y,
          input_cloud->points[i].z;
      beam::Vec2 coords = intrinsics_->ProjectPoint(point);
      uint16_t u = std::round(coords(0, 0)), v = std::round(coords(1, 0));
      tmp.at<cv::Vec3b>(v, u).val[0] = 255;
      tmp.at<cv::Vec3b>(v, u).val[1] = 255;
      tmp.at<cv::Vec3b>(v, u).val[2] = 255;
    }
    cv::dilate(tmp, hit_mask, cv::Mat(dilation, dilation, CV_8UC1),
               cv::Point(-1, -1), 2, 1, 1);
  }

  // This lambda performs ray tracing in parallel on each pixel in the image
  std::mutex mutex;
  image_->forEach<RayTrace::Pixel>([&](RayTrace::Pixel& p,
                                       const int* position) -> void {
    // if the pixel actually traces to a point then continue
    cv::Vec3b colors = hit_mask.at<cv::Vec3b>(position[0], position[1]);
    if (colors.val[0] == 255 && colors.val[1] == 255 && colors.val[2] == 255) {
      // initialize ray
      beam::MatX ray = beam::MatX::Zero(20, 3);
      // get direction between origin and pixel
      beam::Vec3 point;
      double x = position[0] - cy;
      double y = image_->cols - cx - position[1];
      point << x, y, f;
      point.normalize();
      // while loop to ray trace
      uint16_t raypt = 0;
      while (true) {
        // get point at end of ray
        pcl::PointXYZRGB search_point;
        search_point.x = ray(raypt, 0);
        search_point.y = ray(raypt, 1);
        search_point.z = ray(raypt, 2);
        std::lock_guard<std::mutex> lock(mutex);
        // search for closest point to ray
        std::vector<int> point_idx(1);
        std::vector<float> point_distance(1);
        kdtree.nearestKSearch(search_point, 1, point_idx, point_distance);
        float distance = sqrt(point_distance[0]);
        int idx = indices[point_idx[0]];
        // if the point is within 1cm then color it appropriately
        if (distance < 0.01) {
          cloud_colored->points[idx].r = p.z;
          cloud_colored->points[idx].g = p.y;
          cloud_colored->points[idx].b = p.x;
          break;
        } else if (raypt >= 20) {
          break;
        } else {
          raypt++;
          ray(raypt, 0) = ray(raypt - 1, 0) + distance * point(0, 0);
          ray(raypt, 1) = ray(raypt - 1, 1) + distance * point(1, 0);
          ray(raypt, 2) = ray(raypt - 1, 2) + distance * point(2, 0);
        }
      }
    }
  });
  // count the number of points colored
  uint32_t points_colored = 0;
  for (uint32_t i = 0; i < cloud_colored->points.size(); i++) {
    if (cloud_colored->points[i].r != 0 && cloud_colored->points[i].g != 0 &&
        cloud_colored->points[i].b != 0) {
      points_colored++;
    }
  }

  LOG_INFO("Coloured %d of %d total points.", (int)points_colored,
           (int)input_point_cloud_->points.size());
  return cloud_colored;
}

std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, std::vector<int>>
    RayTrace::ReduceCloud_(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
                           std::shared_ptr<cv::Mat> image,
                           beam_calibration::Intrinsics* intrinsics) const {
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

} // namespace beam_colorize
