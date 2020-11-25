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
 * @brief static class implementing various pose estimation algorithm.
 * all p3p code adapted from https://github.com/vlarsson/lambdatwist/
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
                   std::vector<Eigen::Vector3d> points);

  /**
   * @brief Eigen decomp of a matrix which is known to have a 0 eigen value
   * @param x matrix
   * @param E eigenvectors
   * @param L eigenvalues
   */
  static void EigenWithKnownZero(const Eigen::Matrix3d& M, Eigen::Matrix3d& E,
                                 double& sig1, double& sig2);

  /**
   * @brief Gauss-Newton method (least squares by refinement) to find L.  L
   * is a vector of signed distances from the camera center to each 3d point.
   */
  static void RefineLambda(double& lambda1, double& lambda2, double& lambda3,
                           const double a12, const double a13, const double a23,
                           const double b12, const double b13,
                           const double b23);
};

} // namespace beam_cv
