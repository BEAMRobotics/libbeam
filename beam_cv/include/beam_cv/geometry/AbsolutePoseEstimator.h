/** @file
 * @ingroup cv
 */

#pragma once

#include <Eigen/Dense>

#include <beam_calibration/CameraModel.h>
#include <beam_utils/optional.h>

namespace beam_cv {
/**
 * @brief static class implementing various pose estimation algorithm.
 * all p3p code adapted from https://github.com/vlarsson/lambdatwist/
 */
class AbsolutePoseEstimator {
public:
  /**
   * @brief Computes up to 4 possible absolute poses of camera from 3
   * pixel/point correspondences, in no specific order, a 4th point
   * correspondence can be used to remove the ambiguity. Explanation of the
   * generic P3P problem can be found here:
   * https://www-users.cs.umn.edu/~hspark/CSci5980/Lec15_PnP.pdf The algorithm
   * used here however is found here:
   * https://openaccess.thecvf.com/content_ECCV_2018/papers/Mikael_Persson_Lambda_Twist_An_ECCV_2018_paper.pdf
   * @param cam camera model for image
   * @param pixels projected pixel locations of 3 feature points in cam
   * @param points 3d locations of 3 features
   * @returns Up to 4 transformation matrices
   */
  static std::vector<Eigen::Matrix4d>
      P3PEstimator(std::shared_ptr<beam_calibration::CameraModel> cam,
                   std::vector<Eigen::Vector2i> pixels,
                   std::vector<Eigen::Vector3d> points);

  /**
   * @brief RANSAC wrapper for P3PEstimator.  Finds the camera pose with the
   * most inliers of many random samples of 3 correspondences.
   * @param cam camera model for image
   * @param pixels detected pixel locations of features
   * @param points 3d locations of features
   * @param seed to seed the random number generator, default value of -1 will
   * use time as seed
   * @param max_iterations number of ransac iterations to perform
   * @param inlier_threshold pixel distance to count an inlier as
   * @returns Transformation matrix in the reference frame the points are given
   * in
   *
   */
  static Eigen::Matrix4d RANSACEstimator(
      std::shared_ptr<beam_calibration::CameraModel> cam,
      std::vector<Eigen::Vector2i> pixels, std::vector<Eigen::Vector3d> points,
      int max_iterations = 100, double inlier_threshold = 5.0, int seed = -1);

  /**
   * @brief P3P helper -- eigen decomp of a matrix which is known to have a 0
   * eigen value.
   * @param M matrix
   * @param E eigenvectors
   * @param sig1 eigenvalue 1
   * @param sig2 eigenvalue 2
   */
  static void EigenWithKnownZero(const Eigen::Matrix3d& M, Eigen::Matrix3d& E,
                                 double& sig1, double& sig2);

  /**
   * @brief P3P helper -- Gauss-Newton method (least squares by refinement).
   *
   */
  static void RefineLambda(double& lambda1, double& lambda2, double& lambda3,
                           const double a12, const double a13, const double a23,
                           const double b12, const double b13,
                           const double b23);
};

} // namespace beam_cv
