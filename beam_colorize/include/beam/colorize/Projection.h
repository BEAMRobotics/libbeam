#pragma once
#include "beam/colorize/Colorizer.h"

namespace beam_colorize {

class Projection : public Colorizer {
public:
  Projection() = default;

  ~Projection() override = default;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      ColorizePointCloud(const pcl::PointCloud<pcl::PointXYZ>& input_cloud,
                         const sensor_msgs::Image& input_image) const override;

private:

  /**
   * Projects point in camera coordinate system into the natural rectified
   * image.
   * @param cam
   * @param x
   * @param y
   * @param z
   * @return
   */
  std::pair<double, double> XYZToUVRect(int cam, double x, double y,
                                        double z) const;
};

} // namespace beam_colorize