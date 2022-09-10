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
   * @brief see Colorizer.h
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      ColorizePointCloud() const override;

  /**
   * @brief see Colorizer.h
   */
  pcl::PointCloud<beam_containers::PointBridge>::Ptr
      ColorizeMask(bool return_in_cam_frame = false) const override;

private:
};
/** @} group colorizer */

} // namespace beam_colorize
