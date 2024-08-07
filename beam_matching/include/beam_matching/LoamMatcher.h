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

#include <beam_utils/pointclouds.h>

#include <beam_matching/Matcher.h>
#include <beam_matching/loam/LoamParams.h>
#include <beam_matching/loam/LoamPointCloud.h>
#include <beam_matching/loam/LoamScanRegistration.h>

namespace beam_matching {
/** @addtogroup matching
 *  @{ */

/**
 * @brief derived Matcher class which wraps LoamScanRegistration to conform to
 * the Matcher base class interface.
 */
class LoamMatcher : public Matcher<LoamPointCloudPtr> {
public:
  using Params = LoamParams;

  LoamMatcher();

  explicit LoamMatcher(const LoamParams& params);

  ~LoamMatcher();

  /**
   * @brief sets the parameters for the matcher
   * @param params - LoamMatcherParams
   */
  void SetParams(const LoamParams& params);

  /**
   * @brief sets the reference pointcloud for the matcher
   * @param ref - Pointcloud
   */
  void SetRef(const LoamPointCloudPtr& ref);

  /**
   * @brief sets the target (or scene) pointcloud for the matcher
   * @param target - Pointcloud
   */
  void SetTarget(const LoamPointCloudPtr& target);

  /**
   * @brief runs the matcher, blocks until finished.
   * @return true if successful
   */
  bool Match() override;

  /**
   * @brief gets the parameters for the matcher
   * @return LoamMatcherParams
   */
  LoamParamsPtr GetParams() { return params_; }

  /**
   * @brief see matcher.h for details
   */
  void SaveResults(const std::string& output_dir,
                   const std::string& prefix = "cloud") override;

private:
  /**
   * @brief Uses covariance estimate from Ceres
   */
  void CalculateCovariance() override;

  LoamPointCloudPtr ref_;
  LoamPointCloudPtr target_;

  LoamParamsPtr params_;

  std::unique_ptr<LoamScanRegistration> loam_scan_registration_;
};

/** @} group matching */
} // namespace beam_matching
