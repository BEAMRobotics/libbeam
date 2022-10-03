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
  ProjectionMap CreateProjectionMap(
      const PointCloudCol::Ptr& cloud_in_camera_frame) const override;

  /**
   * @brief see Colorizer.h
   */
  ProjectionMap CreateProjectionMap(
      const DefectCloud::Ptr& cloud_in_camera_frame) const override;

  void SetWindowSize(uint8_t window_size);

  void SetWindowStride(uint8_t window_stride);

  void SetDepthThreshold(double depth_seg_thresh_m);

private:
  struct ProjectedPoint {
    uint64_t u;
    uint64_t v;
    uint64_t id;
    double depth;
  };

  ProjectionMap
      RemoveOccludedPointsFromMap(ProjectionMap& projection_map_orig) const;

  void CheckOcclusionsInWindow(ProjectionMap& projection_map_to_keep,
                               ProjectionMap& projection_map, uint64_t u_start,
                               uint64_t v_start) const;

  uint8_t window_size_{90};
  uint8_t window_stride_{67};
  double depth_seg_thresh_m_{0.15};
};
/** @} group colorizer */

} // namespace beam_colorize
