/** @file
 * @ingroup colorizer
 */

#pragma once

#include "beam_colorize/Colorizer.h"

namespace beam_colorize {
/** @addtogroup colorizer
 *  @{ */

/**
 * @brief class for storing a point projection map. This is stored as a 2D (or
 * two level nested) hash map so we can lookup point IDs associated with image
 * pixel coordinates (u,v)
 *
 */
class ProjectionMap {
public:
  ProjectionMap() = default;

  void Add(uint64_t u, uint64_t v, uint64_t point_id);

  const std::vector<uint64_t>& Get(uint64_t u, uint64_t v);

private:
  // map: v -> {map: u -> vector<point IDs>}
  std::unordered_map<uint64_t,
                     std::unordered_map<uint64_t, std::vector<uint64_t>>>
      map_;
}

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
      ColorizeMask() const override;

private:
  ProjectionMap map_;
};
/** @} group colorizer */

} // namespace beam_colorize
