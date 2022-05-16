#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <beam_colorize/RayTraceGpu.h>

namespace beam_colorize {

RayTraceGpu::RayTraceGpu() : Colorizer() {}

void RayTraceGpu::CheckInputConditions();

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RayTraceGpu::ColorizePointCloud() const {
  CheckInputConditions();
  auto cloud_colored = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

  // TODO

  return cloud_colored;
}

PointCloudBridge::Ptr RayTraceGpu::ColorizeMask() const {
  CheckInputConditions();
  auto defect_cloud = std::make_shared<PointCloudBridge>();

  // TODO

  return defect_cloud;
}

} // namespace beam_colorize
