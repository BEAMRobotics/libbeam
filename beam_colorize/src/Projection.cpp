#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include "beam_colorize/Projection.h"

namespace beam_colorize {

Projection::Projection() : Colorizer() {}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Projection::ColorizePointCloud() const {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*input_point_cloud_, *cloud_colored);

  if (!image_initialized_ || !point_cloud_initialized_ ||
      !intrinsics_initialized_ || input_point_cloud_->size() == 0) {
    throw std::runtime_error{"Colorizer not properly initialized."};
    return cloud_colored;
    BEAM_CRITICAL("Colorizer not properly initialized.");
  }

  beam::Vec3 point;
  cv::Vec3b colors;
  beam::Vec2 coords;
  uint16_t u, v;
  uint16_t vmax = image_->cols;
  uint16_t umax = image_->rows;
  uchar blue, green, red;
  int counter = 0;

  for (uint32_t i = 0; i < input_point_cloud_->points.size(); i++) {
    point(0, 0) = input_point_cloud_->points[i].x;
    point(1, 0) = input_point_cloud_->points[i].y;
    point(2, 0) = input_point_cloud_->points[i].z;
    if (point(2, 0) < 0) {
      continue; // make sure points aren't behind image plane
    }
    coords = intrinsics_->ProjectPoint(point);
    u = std::round(coords(0, 0));
    v = std::round(coords(1, 0));
    if (u > 0 && v > 0 && v < vmax && u < umax) {
      colors = image_->at<cv::Vec3b>(u, v);
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
  BEAM_INFO("Coloured {} of {} total points.", counter,
            input_point_cloud_->points.size());
  return cloud_colored;
}

pcl::PointCloud<beam_containers::PointBridge>::Ptr
    Projection::ColorizeMask() const {
  pcl::PointCloud<beam_containers::PointBridge>::Ptr defect_cloud(
      new pcl::PointCloud<beam_containers::PointBridge>);

  pcl::copyPointCloud(*input_point_cloud_, *defect_cloud);

  if (!image_initialized_ || !point_cloud_initialized_ ||
      !intrinsics_initialized_) {
    return defect_cloud;
    throw std::runtime_error{"Colorizer not properly initialized."};
    BEAM_CRITICAL("Colorizer not properly initialized.");
  }

  beam::Vec3 point;
  beam::Vec2 coords;
  uint16_t u, v;
  uint16_t vmax = image_->cols;
  uint16_t umax = image_->rowss;
  int counter = 0;
  uchar color_scale;

  for (uint32_t i = 0; i < input_point_cloud_->points.size(); i++) {
    point(0, 0) = input_point_cloud_->points[i].x;
    point(1, 0) = input_point_cloud_->points[i].y;
    point(2, 0) = input_point_cloud_->points[i].z;
    if (point(2, 0) < 0) {
      continue; // make sure points aren't behind image plane
    }

    coords = intrinsics_->ProjectPoint(point);

    u = std::round(coords(0, 0));
    v = std::round(coords(1, 0));
    if (u > 0 && v > 0 && v < vmax && u < umax) {
      color_scale = image_->at<uchar>(u, v);
      // ignore black colors, this happens at edges when images are undistored
      //      std::cout << "Color scale = " << color_scale << std::endl;
      if (color_scale == 0) {
        continue;
      } else if (color_scale == 1) {
        defect_cloud->points[i].crack = 1;
        counter++;
      } else if (color_scale == 2) {
        defect_cloud->points[i].delam = 1;
        counter++;
      } else if (color_scale == 3) {
        defect_cloud->points[i].corrosion = 1;
        counter++;
      }
    }
  }
  BEAM_INFO("Coloured {} of {} total points.", counter,
            input_point_cloud_->points.size());
  return defect_cloud;
}

} // namespace beam_colorize
