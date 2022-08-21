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
   * @param return_in_cam_frame set to false to return the cloud in its original
   * frame. Internally, when a cloud is added it's converted to the camera
   * frame, so here when we return the pointcloud it can be in either frame.
   * @return Colored point cloud pointer
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      ColorizePointCloud(bool return_in_cam_frame = false) const override;

  /**
   * @brief see Colorizer.h
   */
  pcl::PointCloud<beam_containers::PointBridge>::Ptr
      ColorizeMask(bool return_in_cam_frame = false) const override;

private:
};
/** @} group colorizer */

} // namespace beam_colorize
