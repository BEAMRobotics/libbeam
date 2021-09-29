/** @file
 * @ingroup utils
 *
 * class for making simple 3D paths where motion is predominently in the x
 * direction.
 *
 */

#pragma once

#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/Splines>

#include <beam_utils/math.h>

namespace beam {
/** @addtogroup utils
 *  @{ */

/**
 * @brief this path generator creates a 3D path with motion in the x-y plane. It
 * works by using Eigen to fit a spline to a set of 3D points, then based on
 * your query point (between 0 and 1), it calculates the interpolated point and
 * the yaw to build the 6DOF pose. The yaw is calculated by taking an
 * interpolated point right after the query point and calculating the direction
 * of the spline.
 *
 */
class SimplePathGenerator {
public:
  /**
   * @brief delete default constructor
   */
  SimplePathGenerator() = delete;

  /**
   * @brief main constructor
   * @param nodes set of points used to create spline path
   * @param delta optional change in p to use to calculate direction (yaw)
   */
  SimplePathGenerator(
      const std::vector<Eigen::Vector3d,
                        beam::AlignVec3d>& nodes,
      double delta = 0.001);

  /**
   * @brief interpolate point and return 6DOF pose
   * @param p interpolation point, between 0 and 1
   * @return pose (usually T_WORLD_BASELINK)
   */
  Eigen::Matrix4d GetPose(double p);

private:
  Eigen::Spline3d spline_;
  double delta_;
};

/** @} group utils */
} // namespace beam
