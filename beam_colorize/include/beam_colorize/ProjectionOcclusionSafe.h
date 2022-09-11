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
 * functionality using a safe projection method. This method essentially
 * projects all points the the image, then runs a convolution on the image and
 * checks if there's a large distance discrepancy between the points, if so, it
 * removes the far points. This also helps remove the colorizing error that's
 * common to the perimeter of objects due to poor calibrations or slightly
 * incorrect SLAM poses
 */
class ProjectionOcclusionSafe : public Colorizer {
public:
  ProjectionOcclusionSafe();

  ~ProjectionOcclusionSafe() override = default;

  /**
   * @brief see Colorizer.h
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColorizePointCloud() const override;

  /**
   * @brief see Colorizer.h
   */
  pcl::PointCloud<beam_containers::PointBridge>::Ptr
      ColorizeMask(bool return_in_cam_frame = false) const override;

private:
};
/** @} group colorizer */

} // namespace beam_colorize
