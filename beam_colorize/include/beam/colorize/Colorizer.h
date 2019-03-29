#pragma once

// PCL
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <sensor_msgs/Image.h>

namespace beam_colorize {

class Colorizer {
public:
  Colorizer() = default;

  virtual ~Colorizer() = default;

  /**
   * @brief Method for colorizing a point cloud from an image
   * @param input_cloud Input point cloud in the camera frame
   * @param input_image Image which will be used to color point cloud
   * @return Colored point cloud
   */
  virtual pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      ColorizePointCloud(const pcl::PointCloud<pcl::PointXYZ>& input_cloud,
                         const sensor_msgs::Image& input_image) const = 0;

private:
};

} // namespace beam_colorize