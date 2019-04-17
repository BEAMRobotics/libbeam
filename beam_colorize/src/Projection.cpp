#include "beam/colorize/Projection.h"

namespace beam_colorize {

Projection::Projection() {
  image_distored_ = true;
  image_initialized_ = false;
  point_cloud_initialized_ = false;
  intrinsics_initialized_ = false;
  transform_set_ = false;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Projection::ColorizePointCloud() const {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*input_point_cloud_, *cloud_colored);

  if (!image_initialized_ || !point_cloud_initialized_ ||
      !intrinsics_initialized_) {
    return cloud_colored;
    throw std::runtime_error{"Colorizer not properly initialized."};
    LOG_ERROR("Colorizer not properly initialized.");
  }

  beam::Vec3 point;
  cv::Vec3b colors;
  beam::Vec2 coords;
  uint16_t u, v;
  uint16_t vmax = image_->rows;
  uint16_t umax = image_->cols;
  uchar blue, green, red;
  int counter = 0;

  for (uint32_t i = 0; i < input_point_cloud_->points.size(); i++) {
    point(0, 0) = input_point_cloud_->points[i].x;
    point(1, 0) = input_point_cloud_->points[i].y;
    point(2, 0) = input_point_cloud_->points[i].z;
    if (point(2, 0) < 0) {
      continue; // make sure points aren't behind image plane
    }
    if (image_distored_) {
      coords = intrinsics_->ProjectDistortedPoint(point);
    } else {
      coords = intrinsics_->ProjectPoint(point);
    }
    u = std::round(coords(0, 0));
    v = std::round(coords(1, 0));
    if (u > 0 && v > 0 && v < vmax && u < umax) {
      colors = image_->at<cv::Vec3b>(v, u);
      blue = colors.val[0];
      green = colors.val[1];
      red = colors.val[2];
      // ignore black colors, this happens at edges when images are undistored
      if (red == 0 && green == 0 && blue == 0) {
        continue;
      } else {
        counter++;
        cloud_colored->points[i].r = red;
        cloud_colored->points[i].g = green;
        cloud_colored->points[i].b = blue;
      }
    }
  }
  LOG_INFO("Coloured %d of %d total points.", (int)counter,
           (int)input_point_cloud_->points.size());
  return cloud_colored;
}

} // namespace beam_colorize
