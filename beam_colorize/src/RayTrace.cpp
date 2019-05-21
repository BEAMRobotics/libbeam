#include "beam_colorize/RayTrace.h"

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

pcl::PointCloud<beam_containers::PointBridge>::Ptr
    RayTrace::ColorizeMask() const {
  return pcl::PointCloud<beam_containers::PointBridge>::Ptr();
}

} // namespace beam_colorize
