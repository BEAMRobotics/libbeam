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
   * @brief Computes up to 4 possible absolute poses of camera from the
   * projection of 3 points in the image plane.
   * @param cam camera model for image
   * @param pixels projected pixel locations of 3 feature points in cam
   * @param points 3d locations of 3 features
   * @param cubic_iterations # iterations passed to solve cubic.
   * @param refinement_iterations # iterations passed to refinement.
   * @returns Up to 4 transformation matrices
   */
  static std::vector<Eigen::Matrix4d>
      P3PEstimator(std::shared_ptr<beam_calibration::CameraModel> cam,
                   std::vector<Eigen::Vector2i> pixels,
                   std::vector<Eigen::Vector3d> points,
                   int cubic_iterations = 50, int refinement_iterations = 5);

  /**
   * @brief Finds a single root of the cubic polynomial equation, optimized for
   * the most stable choice.
   * @param b coefficient of cubic
   * @param c coefficient of cubic
   * @param d coefficient of cubic
   * @param iterations number of iterations used to resolve the root
   * @return Cubic root picked for stability.
   *
   */
  static double SolveCubic(double b, double c, double d, int iterations);

  /**
   * @brief Eigen decomp of a matrix which is known to have a 0 eigen value
   * @param b
   * @param c
   * @param r1
   * @param r2
   */
  static bool Root2Real(double b, double c, double& r1, double& r2);

  /**
   * @brief Eigen decomp of a matrix which is known to have a 0 eigen value
   * @param x matrix
   * @param E eigenvectors
   * @param L eigenvalues
   */
  static void EigenWithKnownZero(Eigen::Matrix3d x, Eigen::Matrix3d E,
                                 Eigen::Vector3d L);

  /**
   * @brief Gauss-Newton method (least squares by refinement) to find L.  L
   * is a vector of signed distances from the camera center to each 3d point.
   */
  static void GaussNewtonRefine(Eigen::Vector3d& L, double a12, double a13,
                                double a23, double b12, double b13, double b23,
                                int iterations);
};

} // namespace beam_cv
