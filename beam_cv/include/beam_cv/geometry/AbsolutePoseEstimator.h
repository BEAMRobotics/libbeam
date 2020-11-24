/** @file
 * @ingroup cv
 */

#pragma once

#include <Eigen/Dense>

#include <beam_calibration/CameraModel.h>

template <class T>
using opt = std::optional<T>;

namespace beam_cv {
/**
 * @brief static class implementing various pose estimation algorithms
 */
class AbsolutePoseEstimator {
public:
  /**
   * @brief Computes all possible absolute poses of camera from the projection
   * of 3 points in the image plane.
   * @param cam camera model for image
   * @param pixels projected pixel locations of points in cam (min size = 3)
   * @param points 3d feature locations (min size = 3)
   * @returns Up to 4 transformation matrices
   */
  static std::vector<Eigen::Matrix4d>
      P3PEstimator(std::shared_ptr<beam_calibration::CameraModel> cam,
                   std::vector<Eigen::Vector2i> pixels,
                   std::vector<Eigen::Vector3d> points);

  /**
   * @brief This function finds a single root of the cubic polynomial
   * equation.
   * @param b coefficient of cubic
   * @param c coefficient of cubic
   * @param d coefficient of cubic
   * @param
   * @return Cubic root picked for stability.
   *
   */
  static double SolveCubic(double b, double c, double d, int iterations = 50);

  static bool Root2Real(double b, double c, double& r1, double& r2);

  /**
   * @brief Eigen decomp of a matrix which is known to have a 0 eigen value
   * @param x matrix
   * @param E eigenvectors
   * @param L eigenvalues
   */
  static void EigenWithKnownZero(Eigen::Matrix3d x, Eigen::Matrix3d& E,
                                 Eigen::Vector3d& L);
};

} // namespace beam_cv
