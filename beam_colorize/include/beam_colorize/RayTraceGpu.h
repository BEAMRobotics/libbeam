/** @file
 * @ingroup colorizer
 */

#pragma once

#include <beam_colorize/Colorizer.h>

namespace beam_colorize {
/** @addtogroup colorizer
 *  @{ */

/**
 * @brief Class which implements Colorizer interface and provides colorization
 * functionality using projection methods
 */
class RayTraceGpu : public Colorizer {
public:
  RayTraceGpu();

  ~RayTraceGpu() override = default;

  /**
   * @brief Method for colorizing a point cloud
   * @return Colored point cloud pointer
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColorizePointCloud() const override;

  /**
   * @brief Method for colorizing a point cloud
   * @return Colored point cloud pointer
   */
  PointCloudBridge::Ptr ColorizeMask() const override;

private:
};
/** @} group colorizer */

} // namespace beam_colorize
