/** @file
 * @ingroup matching
 *
 * The is an implementation of Lidar Odometry and Mapping (LOAM). See the
 * following papers:
 *
 *    Zhang, J., & Singh, S. (2014). LOAM : Lidar Odometry and Mapping in
 * Real-time. Robotics: Science and Systems.
 * https://doi.org/10.1007/s10514-016-9548-2
 *
 *    Zhang, J., & Singh, S. (2018). Laser–visual–inertial odometry and mapping
 * with high robustness and low drift. Journal of Field Robotics, 35(8),
 * 1242–1264. https://doi.org/10.1002/rob.21809
 *
 * This code was derived from the following repos:
 *
 *    https://github.com/laboshinl/loam_velodyne
 *
 *    https://github.com/libing64/lidar_pose_estimator
 *
 *    https://github.com/TixiaoShan/LIO-SAM
 *
 */

#pragma once

#include <beam_matching/Matcher.h>
#include <beam_matching/LoamPointCloud.h>
#include <beam_utils/pointclouds.h>

namespace beam_matching {
/** @addtogroup matching
 *  @{ */

struct LoamMatcherParams {
  LoamMatcherParams(std::string& param_config);
  LoamMatcherParams() {}

  /// Voxel side length for downsampling. If set to 0, downsampling is
  /// not performed. If multiscale matching is set, this is the resolution
  /// of the final, fine-scale match
  float res = 0;

  // shared pointer to LoamParams. These are needed inside the LoamPointCloud
  // class so we will store it as a pointer to a separate class
  std::shared_ptr<LoamParams> loam_params;
};

class LoamMatcher : public Matcher<PointCloudPtr> {
public:
  LoamMatcher() = default;
  /**
   * @brief This constructor takes an argument in order to adjust how much
   * downsampling is done before matching is attempted. Pointclouds are
   * downsampled using a voxel filter, the argument is the edge length of
   * each voxel. If resolution is non-positive, no downsampling is used.
   */
  explicit LoamMatcher(LoamMatcherParams params);

  ~LoamMatcher();

  /**
   * @brief sets the parameters for the matcher
   * @param params - LoamMatcherParams
   */
  void SetParams(LoamMatcherParams params);

  /**
   * @brief sets the reference pointcloud for the matcher
   * @param ref - Pointcloud
   */
  void SetRef(const PointCloudPtr& ref);

  /**
   * @brief sets the target (or scene) pointcloud for the matcher
   * @param target - Pointcloud
   */
  void SetTarget(const PointCloudPtr& target);

  /**
   * @brief runs the matcher, blocks until finished.
   * @return true if successful
   */
  bool Match();

  /**
   * @brief gets the parameters for the matcher
   * @return LoamMatcherParams
   */
  LoamMatcherParams GetParams() { return params_; }

private:
  void SetLoamParams();

  /** An instance of a PCL voxel filter. It is used to downsample input. */
  pcl::VoxelGrid<pcl::PointXYZ> filter_;

  PointCloudPtr ref_;
  PointCloudPtr target_;
  PointCloudPtr final_;

  LoamMatcherParams params_;
};

/** @} group matching */
} // namespace beam_matching
