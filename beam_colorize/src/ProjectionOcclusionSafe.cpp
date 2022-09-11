#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include "beam_colorize/ProjectionOcclusionSafe.h"

namespace beam_colorize {

ProjectionOcclusionSafe::ProjectionOcclusionSafe() : Colorizer() {}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    ProjectionOcclusionSafe::ColorizePointCloud() const {
  auto cloud_colored = std::make_shared<PointCloudCol>();

  // TODO

  return cloud_colored;
}

pcl::PointCloud<beam_containers::PointBridge>::Ptr
    ProjectionOcclusionSafe::ColorizeMask() const {
  auto defect_cloud = std::make_shared<beam_containers::PointBridge>();

  // TODO

  return defect_cloud;
}

} // namespace beam_colorize
