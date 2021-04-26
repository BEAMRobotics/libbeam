/** @file
 * @ingroup matching
 *
 * Wrapper of GICP in PCL
 * 
 */

#pragma once

#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>

#include <beam_matching/Matcher.h>
#include <beam_utils/pointclouds.h>

namespace beam_matching {
/** @addtogroup matching
 *  @{ */

struct GicpMatcherParams {
  GicpMatcherParams(const std::string& config_path);
  GicpMatcherParams() {}

  int corr_rand = 10;
  int max_iter = 100;
  double r_eps = 1e-8;
  double fit_eps = 1e-2;
  float res = 0.1;
};

class GicpMatcher : public Matcher<PointCloudPtr> {
public:
  GicpMatcher() = default;
  explicit GicpMatcher(const GicpMatcherParams params);

  /**
   * @brief sets the parameters for the matcher
   * @param params - GicpMatcherParams
   */
  void SetParams(const GicpMatcherParams params);

  /**
   * @brief gets the parameters for the matcher
   * @return GicpMatcherParams
   */
  GicpMatcherParams GetParams() { return params_; }

  /**
   * @brief sets the reference pointcloud for the matcher
   * @param ref - Pointcloud
   */
  void SetRef(const PointCloudPtr& ref);

  /**
   * @brief sets the target (or scene) pointcloud for the matcher
   * @param targer - Pointcloud
   */
  void SetTarget(const PointCloudPtr& target);

  /**
   * @briefruns the matcher, blocks until finished.
   * @return true if successful
   */
  bool Match();

private:
  /**
   * @brief configures the gicp matcher
   */
  void SetGicpParams();

  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_;
  pcl::VoxelGrid<pcl::PointXYZ> filter_;
  PointCloudPtr ref_;
  PointCloudPtr target_;
  PointCloudPtr final_;
  GicpMatcherParams params_;
};

/** @} group matching */
} // namespace beam_matching