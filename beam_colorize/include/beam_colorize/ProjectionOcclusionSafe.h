/** @file
 * @ingroup colorizer
 */

#pragma once

#include "beam_colorize/Colorizer.h"

namespace beam_colorize {
/** @addtogroup colorizer
 *  @{ */

using UMapType = std::unordered_map<uint64_t, uint64_t>;

/**
 * @brief class for storing a point projection map. This is stored as a 2D (or
 * two level nested) hash map so we can lookup point IDs associated with image
 * pixel coordinates (u,v). Note that since we only want to colorize closest
 * points to the image, we only store the closest points to each individual
 * pixel
 *
 */
class ProjectionMap {
public:
  explicit ProjectionMap(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in_camera_frame);

  void Add(uint64_t u, uint64_t v, uint64_t point_id);

  bool Get(uint64_t u, uint64_t v, uint64_t& id);

  void Erase(uint64_t u, uint64_t v);

  int Size();

  std::unordered_map<uint64_t, UMapType>::iterator VBegin();

  std::unordered_map<uint64_t, UMapType>::iterator VEnd();

private:
  // map: v -> {map: u -> closest point ID}
  std::unordered_map<uint64_t, UMapType> map_;

  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
};

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

  void SetWindowSize(uint8_t window_size);

  void SetWindowStride(uint8_t window_stride);

  void SetDepthThreshold(double depth_seg_thresh_m);

private:
  struct ProjectedPoint {
    uint64_t u;
    uint64_t v;
    uint64_t id;
  };

  ProjectionMap
      RemoveOccludedPointsFromMap(ProjectionMap& projection_map_orig) const;

  void CheckOcclusionsInWindow(ProjectionMap& projection_map_to_keep,
                               ProjectionMap& projection_map, uint64_t u_start,
                               uint64_t v_start) const;

  uint8_t window_size_{6};
  uint8_t window_stride_{3};
  double depth_seg_thresh_m_{0.1};
};
/** @} group colorizer */

} // namespace beam_colorize
