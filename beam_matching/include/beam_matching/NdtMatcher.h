/** @file
 * @ingroup matching
 *
 * Wrapper of NDT in PCL.
 *
 * There are a few parameters that may be changed specific to this algorithm.
 * They can be set in the json config file. The path to this file should be
 * specified in the constructor for the class. A sample json file can be found
 * in the beam_matching/config directory.
 *
 * - step_size: Maximum Newton step size used in line search
 * - max_iter: Limits number of iterations
 * - t_eps: Criteria to stop iterating. If the difference between consecutive
 * transformations is less than this, stop.
 * - default_res: If the contructor is given an invalid resolution (too fine or
 * negative), use this resolution
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

#include <beam_matching/Matcher.h>
#include <beam_utils/pointclouds.h>

namespace beam_matching {
/** @addtogroup matching
 *  @{ */

class NdtMatcher : public Matcher<PointCloudPtr> {
public:
  struct Params {
    Params(const std::string& config_path);
    Params() {}

    int step_size{3};
    int max_iter{100};
    double t_eps{1e-8};
    float res{5};
    float min_res{0.05f};
  };

  NdtMatcher() = default;

  /**
   * @brief This constructor takes an argument in order to specify resolution.
   * The resolution given takes precedence over the one in the config file. If
   * both resolutions are finer than the `min_res` class member, the
   * resolution is set to `min_res.`
   */
  NdtMatcher(Params params);
  ~NdtMatcher();

  /**
   * @brief sets the parameters for the matcher
   * @param params - NdtMatcher Params
   */
  void SetParams(Params params);

  /**
   * @brief Gets the parameters for the matcher
   * @return NdtMatcher Params
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
   * @brief runs the matcher, blocks until finished.
   * Note that this version of ndt is SLOW
   * Returns true if successful
   */
  bool Match();

private:
  /**
   * @brief configures ndt matcher with the parameters
   */
  void SetNdtParams();

  /** An instance of the NDT class from PCL */
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;

  /** Pointers to the reference and target pointclouds. The "final" pointcloud
   * is not exposed. PCL's NDT class creates an aligned verison of the target
   * pointcloud after matching, so the "final" member is used as a sink for
   * it. */
  PointCloudPtr ref_;
  PointCloudPtr target_;
  PointCloudPtr final_;
  Params params_;
};

using NdtMatcherParams = NdtMatcher::Params;

/** @} group matching */
} // namespace beam_matching
