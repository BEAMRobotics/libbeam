/** @file
 * @ingroup matching
 *
 * Wrapper of NDT in PCL.
 *
 * There are a few parameters that may be changed specific to this algorithm.
 * They can be set in the yaml config file. The path to this file should be
 * specified in the constructor for the class. A sample yaml file can be found
 * in the beam_matching/config directory.
 *
 * - step_size: Maximum Newton step size used in line search
 * - max_iter: Limits number of iterations
 * - t_eps: Criteria to stop iterating. If the difference between consecutive
 * transformations is less than this, stop.
 * - default_res: If the contructor is given an invalid resolution (too fine or
 * negative), use this resolution
 */

#ifndef BEAM_MATCHING_NDT_MATCHER_HPP
#define BEAM_MATCHING_NDT_MATCHER_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>

#include "beam_matching/Matcher.hpp"
#include "beam_matching/pcl_common.hpp"

namespace beam_matching {
/** @addtogroup matching
 *  @{ */

struct NdtMatcherParams {
  NdtMatcherParams(const std::string &config_path);
  NdtMatcherParams() {}

  int step_size = 3;
  int max_iter = 100;
  double t_eps = 1e-8;
  float res = 5;
  float min_res = 0.05f;
};

class NdtMatcher : public Matcher<PCLPointCloudPtr> {
public:
  NdtMatcher() = default;
  /** This constructor takes an argument in order to specify resolution. The
   * resolution given takes precedence over the one in the config file. If
   * both resolutions are finer than the `min_res` class member, the
   * resolution is set to `min_res.`
   */
  explicit NdtMatcher(NdtMatcherParams params);
  ~NdtMatcher();

  /** sets the parameters for the matcher
   * @param params - NdtMatcherParams
   */
  void SetParams(NdtMatcherParams params);

  /** Gets the parameters for the matcher
   * @return NdtMatcherParams
   */
  NdtMatcherParams GetParams() { return params_; }

  /** sets the reference pointcloud for the matcher
   * @param ref - Pointcloud
   */
  void SetRef(const PCLPointCloudPtr &ref);

  /** sets the target (or scene) pointcloud for the matcher
   * @param targer - Pointcloud
   */
  void SetTarget(const PCLPointCloudPtr &target);

  /** runs the matcher, blocks until finished.
   * Note that this version of ndt is SLOW
   * Returns true if successful
   */
  bool Match();

private:
  /**
   * configures ndt matcher with the parameters
   */
  void SetNdtParams();
  /** An instance of the NDT class from PCL */
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;

  /** Pointers to the reference and target pointclouds. The "final" pointcloud
    * is not exposed. PCL's NDT class creates an aligned verison of the target
    * pointcloud after matching, so the "final" member is used as a sink for
    * it. */
  PCLPointCloudPtr ref_, target_, final_;
  NdtMatcherParams params_;
};

/** @} group matching */
}  // namespace beam_matching

#endif  // BEAM_MATCHING_NDT_MATCHER_HPP
