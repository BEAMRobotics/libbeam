/** @file
 * @ingroup matching
 *
 * Wrapper of GICP in PCL
 *
 */

#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>

#include <beam_matching/Matcher.h>
#include <beam_utils/pointclouds.h>

namespace beam_matching {
/** @addtogroup matching
 *  @{ */

class GicpMatcher : public Matcher<PointCloudPtr> {
public:
  struct Params {
    Params(const std::string& config_path);
    Params() {}

    int corr_rand{10};
    int max_iter{100};
    double r_eps{1e-8};
    double fit_eps{1e-2};
    float res{0.1};
  };

  GicpMatcher() = default;
  GicpMatcher(const Params params);

  /**
   * @brief sets the parameters for the matcher
   * @param params - GicpMatcher Params
   */
  void SetParams(const Params params);

  /**
   * @brief gets the parameters for the matcher
   * @return GicpMatcher Params
   */
  Params GetParams() { return params_; }

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
  bool Match() override;

  /**
   * @brief see matcher.h for details
   */
  void SaveResults(const std::string& output_dir,
                   const std::string& prefix = "cloud") override;

  /**
   * @brief Gets the resulting fitness score
   */
  double GetFitnessScore() { return gicp_.getFitnessScore(); }

private:
  /**
   * @brief configures the gicp matcher
   */
  void SetGicpParams();

  /**
   * @brief Not yet implemented, will throw exception if called
   */
  void CalculateCovariance() override;

  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_;
  pcl::VoxelGrid<pcl::PointXYZ> filter_;
  PointCloudPtr ref_;
  PointCloudPtr target_;
  PointCloudPtr final_;
  Params params_;
};

using GicpMatcherParams = GicpMatcher::Params;

/** @} group matching */
} // namespace beam_matching
