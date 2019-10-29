/** @file
 * @ingroup matching
 *
 * Wrapper of ICP in PCL
 *
 * There are a few parameters that may be changed specific to this algorithm.
 * They can be set in the yaml config file.
 *
 * - max_corr: correspondences behind this many distance-units are
 * discarded
 * - max_iter: Limits number of ICP iterations
 * - t_eps: Criteria to stop iterating. If the difference between consecutive
 * transformations is less than this, stop.
 * - fit_eps: Criteria to stop iterating. If the cost function does not improve
 * by more than this quantity, stop.
 */

#ifndef BEAM_MATCHING_ICP_MATCHER_HPP
#define BEAM_MATCHING_ICP_MATCHER_HPP

#include <pcl/registration/icp.h>

#include "beam_matching/Matcher.hpp"
#include "beam_matching/pcl_common.hpp"

namespace beam_matching {
/** @addtogroup matching
 *  @{ */

struct IcpMatcherParams {
  IcpMatcherParams(std::string& param_config);
  IcpMatcherParams() {}

  /// Maximum distance to correspond points for icp
  double max_corr = 3;
  /// Maximum iterations of ICP
  int max_iter = 100;
  /// Transformation epsilon. Stopping criteria. If the transform changes by
  /// less
  /// than this amount, stop
  double t_eps = 1e-8;
  /// Stopping criteria, if cost function decreases by less than this, stop
  double fit_eps = 1e-2;
  /// Angular variance for lidar sensor model. Used if Censi covariance
  /// estimation is set
  double lidar_ang_covar = 7.78e-9;
  /// Linear variance for lidar sensor model. Used if Censi covariance
  /// estimation is set
  double lidar_lin_covar = 2.5e-4;

  /// When set to more than 0, each match is performed multiple times from
  /// a coarse to fine scale (in terms of voxel downsampling).
  /// Each step doubles the resolution
  int multiscale_steps = 3;

  /// Voxel side length for downsampling. If set to 0, downsampling is
  /// not performed. If multiscale matching is set, this is the resolution
  /// of the final, fine-scale match
  float res = 0.1;
  enum covar_method : int {
    LUM,
    CENSI,
    LUMold
  } covar_estimator = covar_method::LUM;
};

class IcpMatcher : public Matcher<PCLPointCloudPtr> {
public:
  IcpMatcher() = default;
  /** This constructor takes an argument in order to adjust how much
   * downsampling is done before matching is attempted. Pointclouds are
   * downsampled using a voxel filter, the argument is the edge length of
   * each voxel. If resolution is non-positive, no downsampling is used.
   */
  explicit IcpMatcher(IcpMatcherParams params);
  ~IcpMatcher();

  /** sets the parameters for the matcher
   * @param params - IcpMatcherParams
   */
  void SetParams(IcpMatcherParams params);

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

  /** runs covariance estimator, blocks until finished.
   */
  void EstimateInfo();

  /** gets the parameters for the matcher
   * @return IcpMatcherParams
   */
  IcpMatcherParams GetParams() { return params_; }

private:
  /** An instance of the ICP class from PCL */
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
  /** An instance of a PCL voxel filter. It is used to downsample input. */
  pcl::VoxelGrid<pcl::PointXYZ> filter_;

  /** Pointers to the reference and target pointclouds. The "final" pointcloud
   * is not exposed. PCL's ICP class creates an aligned verison of the target
   * pointcloud after matching, so the "final" member is used as a sink for
   * it. */
  PCLPointCloudPtr ref_, target_, final_, downsampled_ref_, downsampled_target_;

  IcpMatcherParams params_;

  /**
   * Calculates a covariance estimate based on Lu and Milios Scan Matching
   */
  void EstimateLUM();
  void EstimateLUMold();

  /**
   * Calculates a covariance estimate based on Censi
   */
  void EstimateCensi();

  /**
   * configures icp matcher with the parameters
   */
  void SetIcpParams();
};

/** @} group matching */
} // namespace beam_matching

#endif // BEAM_MATCHING_ICP_MATCHER_HPP
