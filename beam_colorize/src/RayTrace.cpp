#include "beam/colorize/RayTrace.h"

namespace beam_colorize {

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RayTrace::ColorizePointCloud(
    const pcl::PointCloud<pcl::PointXYZ>& input_cloud,
    const sensor_msgs::Image& input_image) const {

  return pcl::PointCloud<pcl::PointXYZRGB>::Ptr();
}

} // namespace beam_colorize