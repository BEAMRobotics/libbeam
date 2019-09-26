/** @file
 * @ingroup matching
 *
 * Wrapper of ICP in PCL
 *
 * There are a few parameters that may be changed specific to this algorithm.
 * They can be set in the yaml config file.
 *
 * - corr_rand: nearest neighbour correspondences used to calculate
 * distributions
 * - max_iter: Limits number of ICP iterations
 * - t_eps: Criteria to stop iterating. If the difference between consecutive
 * transformations is less than this, stop.
 * - fit_eps: Criteria to stop iterating. If the cost function does not improve
 * by more than this quantity, stop.
 */

#ifndef BEAM_MATCHING_GICP_HPP
#define BEAM_MATCHING_GICP_HPP

#include <pcl/registration/gicp.h>

#include "beam_matching/Matcher.hpp"
#include "beam_matching/pcl_common.hpp"

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

class GicpMatcher : public Matcher<PCLPointCloudPtr> {
public:
  GicpMatcher() = default;
  explicit GicpMatcher(const GicpMatcherParams params);

  /** sets the parameters for the matcher
   * @param params - GicpMatcherParams
   */
  void SetParams(const GicpMatcherParams params);

  /** gets the parameters for the matcher
   * @return GicpMatcherParams
   */
  GicpMatcherParams GetParams() { return params_; }

  /** sets the reference pointcloud for the matcher
   * @param ref - Pointcloud
   */
  void SetRef(const PCLPointCloudPtr& ref);

  /** sets the target (or scene) pointcloud for the matcher
   * @param targer - Pointcloud
   */
  void SetTarget(const PCLPointCloudPtr& target);

  /** runs the matcher, blocks until finished.
   * Returns true if successful
   */
  bool Match();

private:
  /**
   * configures the gicp matcher
   */
  void SetGicpParams();
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_;
  pcl::VoxelGrid<pcl::PointXYZ> filter_;
  PCLPointCloudPtr ref_, target_, final_;
  GicpMatcherParams params_;
};

/** @} group matching */
} // namespace beam_matching

#endif // BEAM_MATCHING_ICP_HPP
