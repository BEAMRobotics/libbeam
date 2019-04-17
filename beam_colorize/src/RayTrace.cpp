#include "beam/colorize/RayTrace.h"

namespace beam_colorize {

RayTrace::RayTrace() {
  image_distored_ = true;
  image_initialized_ = false;
  point_cloud_initialized_ = false;
  intrinsics_initialized_ = false;
  transform_set_ = false;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RayTrace::ColorizePointCloud() const {
  return pcl::PointCloud<pcl::PointXYZRGB>::Ptr();
}

} // namespace beam_colorize
