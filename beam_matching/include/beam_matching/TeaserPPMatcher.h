/** @file
 * @ingroup matching
 *
 * Wrapper of Teaser++
 * See: https://github.com/MIT-SPARK/TEASER-plusplus/blob/master/doc/quickstart.rst
 * 
 */

#pragma once

#include <pcl/filters/voxel_grid.h>
#include <teaser/registration.h>

#include <beam_matching/Matcher.h>
#include <beam_utils/pointclouds.h>

namespace beam_matching {
/** @addtogroup matching
 *  @{ */

struct TeaserPPMatcherParams {
  TeaserPPMatcherParams(const std::string& config_path);
  TeaserPPMatcherParams() {}

  using RotAlgo = teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM;

  double noise_bound = 0.05;
  double cbar2 = 1;
  bool estimate_scaling = false;
  int rotation_max_iterations = 100;
  double rotation_gnc_factor = 1.4;
  RotAlgo rotation_estimation_algorithm = RotAlgo::GNC_TLS;
  double rotation_cost_threshold = 0.005;
  float res = 0.1;

  teaser::RobustRegistrationSolver::Params GetSolverParams();
  
};

class TeaserPPMatcher : public Matcher<PointCloudPtr> {
public:

  TeaserPPMatcher() = default;
  explicit TeaserPPMatcher(const TeaserPPMatcherParams params);

  /**
   * @brief sets the parameters for the matcher
   * @param params - TeaserPPMatcherParams
   */
  void SetParams(const TeaserPPMatcherParams params);

  /**
   * @brief gets the parameters for the matcher
   * @return TeaserPPMatcherParams
   */
  TeaserPPMatcherParams GetParams() { return params_; }

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
   * @brief configures the matcher
   */
  void SetTeaserPPParams();

  /**
   * @brief return the ref cloud transformed using the scan registration results
   * @return transformed cloud pointer
   */
  PointCloudPtr GetAlignedRefCloud();

  teaser::RobustRegistrationSolver teaserpp_;
  pcl::VoxelGrid<pcl::PointXYZ> filter_;
  PointCloudPtr ref_;
  PointCloudPtr target_;
  TeaserPPMatcherParams params_;
};

/** @} group matching */
} // namespace beam_matching
