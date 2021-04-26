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

  double noise_bound{0.05};
  double cbar2{1};
  bool estimate_scaling{false};
  int rotation_max_iterations{100};
  double rotation_gnc_factor{1.4};
  RotAlgo rotation_estimation_algorithm{RotAlgo::GNC_TLS};
  double rotation_cost_threshold{0.005};
  float res{0.1};
  bool estimate_correspondences{true};
  float corr_normal_search_radius{0.02};
  float corr_fpfh_search_radius{0.04};

  teaser::RobustRegistrationSolver::Params GetSolverParams();
  
};

/**
 * @brief Teaser++ Wrapper around 
 *   https://github.com/MIT-SPARK/TEASER-plusplus
 * NOTE: this implementation requires input clouds to be in order
 * based on their estimated correspondences. I.e., the ith point 
 * of the ref cloud must correspond to the ith point of the target
 * cloud. This is an oversight in the interface of Teaser++. 
 * @TODO: Calculate the correspondences (see note above)
 */
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
   * @brief runs the matcher, blocks until finished.
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
