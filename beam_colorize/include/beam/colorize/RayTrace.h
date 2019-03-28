#pragma once
#include "beam/colorize/Colorizer.h"

namespace beam_colorize {

class RayTrace : public Colorizer {
public:
  RayTrace() = default;

  ~RayTrace() = default;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      ColorizePointCloud(const pcl::PointCloud<pcl::PointXYZ>& input_cloud,
                         const sensor_msgs::Image& input_image) const override;

private:
};
} // namespace beam_colorize