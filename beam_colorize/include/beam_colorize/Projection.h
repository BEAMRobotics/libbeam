/** @file
 * @ingroup colorizer
 */

#pragma once
#include "beam_colorize/Colorizer.h"

namespace beam_colorize {
/** @addtogroup colorizer
 *  @{ */

/**
 * @brief Class which implements Colorizer interface and provides colorization
 * functionality using projection methods
 */
class Projection : public Colorizer {
public:
  Projection();

  ~Projection() override = default;

  /**
   * @brief Method for colorizing a point cloud
   * @return Colored point cloud pointer
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      ColorizePointCloud(int dilation = -1) const override;

private:
};
/** @} group colorizer */

} // namespace beam_colorize
