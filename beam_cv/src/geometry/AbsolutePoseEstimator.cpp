#include "beam_cv/geometry/AbsolutePoseEstimator.h"

#include <Eigen/Geometry>

#include "beam_cv/geometry/Triangulation.h"
#include <beam_cv/Utils.h>
#include <beam_utils/math.h>
#include <numeric>

namespace beam_cv {
std::vector<Eigen::Matrix4d> AbsolutePoseEstimator::P3PEstimator(
    std::shared_ptr<beam_calibration::CameraModel> cam,
    std::vector<Eigen::Vector2i> pixels, std::vector<Eigen::Vector3d> points) {
  // store normalized pixels coords
  std::vector<Eigen::Vector3d> x;
  for (std::size_t i = 0; i < pixels.size(); ++i) {
    // back project and normalize each pixel before adding it to x
    Eigen::Vector3d homogenized_pixel = cam->BackProject(pixels[i]).value();
    homogenized_pixel.normalize();
    x.push_back(homogenized_pixel);
  };

  // lambdatwist p3p implementation adapted from
  // https://github.com/vlarsson/lambdatwist
  Eigen::Vector3d dX12 = points[0] - points[1];
  Eigen::Vector3d dX13 = points[0] - points[2];
  Eigen::Vector3d dX23 = points[1] - points[2];

  double a12 = dX12.squaredNorm();
  double b12 = x[0].dot(x[1]);

  double a13 = dX13.squaredNorm();
  double b13 = x[0].dot(x[2]);

  double a23 = dX23.squaredNorm();
  double b23 = x[1].dot(x[2]);

  double a23b12 = a23 * b12;
  double a12b23 = a12 * b23;
  double a23b13 = a23 * b13;
  double a13b23 = a13 * b23;

  Eigen::Matrix3d D1, D2;

  D1 << a23, -a23b12, 0.0, -a23b12, a23 - a12, a12b23, 0.0, a12b23, -a12;
  D2 << a23, 0.0, -a23b13, 0.0, -a13, a13b23, -a23b13, a13b23, a23 - a13;

  Eigen::Matrix3d DX1, DX2;
  DX1 << D1.col(1).cross(D1.col(2)), D1.col(2).cross(D1.col(0)),
      D1.col(0).cross(D1.col(1));
  DX2 << D2.col(1).cross(D2.col(2)), D2.col(2).cross(D2.col(0)),
      D2.col(0).cross(D2.col(1));

  // Coefficients of p(gamma) = det(D1 + gamma*D2)
  // In the original paper c2 and c1 are switched.
  double c3 = D2.col(0).dot(DX2.col(0));
  double c2 = (D1.array() * DX2.array()).sum();
  double c1 = (D2.array() * DX1.array()).sum();
  double c0 = D1.col(0).dot(DX1.col(0));

  // closed root solver for cubic root
  const double c3inv = 1.0 / c3;
  c2 *= c3inv;
  c1 *= c3inv;
  c0 *= c3inv;

  double a = c1 - c2 * c2 / 3.0;
  double b = (2.0 * c2 * c2 * c2 - 9.0 * c2 * c1) / 27.0 + c0;
  double c = b * b / 4.0 + a * a * a / 27.0;
  double gamma;
  if (c > 0) {
    c = std::sqrt(c);
    b *= -0.5;
    gamma = std::cbrt(b + c) + std::cbrt(b - c) - c2 / 3.0;
  } else {
    c = 3.0 * b / (2.0 * a) * std::sqrt(-3.0 / a);
    gamma = 2.0 * std::sqrt(-a / 3.0) * std::cos(std::acos(c) / 3.0) - c2 / 3.0;
  }

  // We do a single newton step on the cubic equation
  double f = gamma * gamma * gamma + c2 * gamma * gamma + c1 * gamma + c0;
  double df = 3.0 * gamma * gamma + 2.0 * c2 * gamma + c1;
  gamma = gamma - f / df;

  Eigen::Matrix3d D0 = D1 + gamma * D2;

  Eigen::Matrix3d E;
  double sig1, sig2;

  AbsolutePoseEstimator::EigenWithKnownZero(D0, E, sig1, sig2);

  double s = std::sqrt(-sig2 / sig1);

  double w0p = (E(1, 0) - s * E(1, 1)) / (s * E(0, 1) - E(0, 0));
  double w1p = (-s * E(2, 1) + E(2, 0)) / (s * E(0, 1) - E(0, 0));

  double w0n = (E(1, 0) + s * E(1, 1)) / (-s * E(0, 1) - E(0, 0));
  double w1n = (s * E(2, 1) + E(2, 0)) / (-s * E(0, 1) - E(0, 0));

  // Note that these formulas differ from what is presented in the paper.
  double ap = (a13 - a12) * w1p * w1p + 2.0 * a12 * b13 * w1p - a12;
  double bp = -2.0 * a13 * b12 * w1p + 2.0 * a12 * b13 * w0p -
              2.0 * w0p * w1p * (a12 - a13);
  double cp = (a13 - a12) * w0p * w0p - 2.0 * a13 * b12 * w0p + a13;

  double an = (a13 - a12) * w1n * w1n + 2.0 * a12 * b13 * w1n - a12;
  double bn = 2.0 * a12 * b13 * w0n - 2.0 * a13 * b12 * w1n -
              2.0 * w0n * w1n * (a12 - a13);
  double cn = (a13 - a12) * w0n * w0n - 2.0 * a13 * b12 * w0n + a13;

  double lambda1, lambda2, lambda3;

  // prep for output
  std::vector<Eigen::Matrix4d> output;
  Eigen::Matrix4d solution = Eigen::Matrix4d::Zero();
  solution(3, 3) = 1;

  Eigen::Matrix3d XX;

  XX << dX12, dX13, dX12.cross(dX13);
  XX = XX.inverse().eval();

  Eigen::Vector3d v1, v2;
  Eigen::Matrix3d YY;

  double b2m4ac = bp * bp - 4.0 * ap * cp;

  if (b2m4ac > 0) {
    double sq = std::sqrt(b2m4ac);

    // first root
    double tau = (bp > 0) ? (2.0 * cp) / (-bp - sq) : (2.0 * cp) / (-bp + sq);

    if (tau > 0) {
      lambda2 = std::sqrt(a23 / (tau * (tau - 2.0 * b23) + 1.0));
      lambda3 = tau * lambda2;
      lambda1 = w0p * lambda2 + w1p * lambda3;

      if (lambda1 > 0) {
        AbsolutePoseEstimator::RefineLambda(lambda1, lambda2, lambda3, a12, a13,
                                            a23, b12, b13, b23);
        v1 = lambda1 * x[0] - lambda2 * x[1];
        v2 = lambda1 * x[0] - lambda3 * x[2];
        YY << v1, v2, v1.cross(v2);
        solution.block(0, 0, 3, 3) = YY * XX;
        solution.block(0, 3, 3, 1) =
            lambda1 * x[0] - solution.block(0, 0, 3, 3) * points[0];
        output.push_back(solution);
      }
    }

    // Second root
    tau = cp / (ap * tau);

    if (tau > 0) {
      lambda2 = std::sqrt(a23 / (tau * (tau - 2.0 * b23) + 1.0));
      lambda3 = tau * lambda2;
      lambda1 = w0p * lambda2 + w1p * lambda3;

      if (lambda1 > 0) {
        AbsolutePoseEstimator::RefineLambda(lambda1, lambda2, lambda3, a12, a13,
                                            a23, b12, b13, b23);
        v1 = lambda1 * x[0] - lambda2 * x[1];
        v2 = lambda1 * x[0] - lambda3 * x[2];
        YY << v1, v2, v1.cross(v2);
        solution.block(0, 0, 3, 3) = YY * XX;
        solution.block(0, 3, 3, 1) =
            lambda1 * x[0] - solution.block(0, 0, 3, 3) * points[0];
        output.push_back(solution);
      }
    }
  }

  // Second pair of roots
  b2m4ac = bn * bn - 4.0 * an * cn;

  if (b2m4ac > 0) {
    double sq = std::sqrt(b2m4ac);

    // first root
    double tau = (bn > 0) ? (2.0 * cn) / (-bn - sq) : (2.0 * cn) / (-bn + sq);

    if (tau > 0) {
      lambda2 = std::sqrt(a23 / (tau * (tau - 2.0 * b23) + 1.0));
      lambda3 = tau * lambda2;
      lambda1 = w0n * lambda2 + w1n * lambda3;

      if (lambda1 > 0) {
        AbsolutePoseEstimator::RefineLambda(lambda1, lambda2, lambda3, a12, a13,
                                            a23, b12, b13, b23);
        v1 = lambda1 * x[0] - lambda2 * x[1];
        v2 = lambda1 * x[0] - lambda3 * x[2];
        YY << v1, v2, v1.cross(v2);
        solution.block(0, 0, 3, 3) = YY * XX;
        solution.block(0, 3, 3, 1) =
            lambda1 * x[0] - solution.block(0, 0, 3, 3) * points[0];
        output.push_back(solution);
      }
    }

    // Second root
    tau = cn / (an * tau);

    if (tau > 0) {
      lambda2 = std::sqrt(a23 / (tau * (tau - 2.0 * b23) + 1.0));
      lambda3 = tau * lambda2;
      lambda1 = w0n * lambda2 + w1n * lambda3;

      if (lambda1 > 0) {
        AbsolutePoseEstimator::RefineLambda(lambda1, lambda2, lambda3, a12, a13,
                                            a23, b12, b13, b23);
        v1 = lambda1 * x[0] - lambda2 * x[1];
        v2 = lambda1 * x[0] - lambda3 * x[2];
        YY << v1, v2, v1.cross(v2);
        solution.block(0, 0, 3, 3) = YY * XX;
        solution.block(0, 3, 3, 1) =
            lambda1 * x[0] - solution.block(0, 0, 3, 3) * points[0];
        output.push_back(solution);
      }
    }
  }

  return output;
}

Eigen::Matrix4d AbsolutePoseEstimator::RANSACEstimator(
    std::shared_ptr<beam_calibration::CameraModel> cam,
    std::vector<Eigen::Vector2i> pixels, std::vector<Eigen::Vector3d> points,
    int max_iterations, double inlier_threshold, int seed) {
  // return nothing if input sizes do not match
  if (pixels.size() != points.size()) {
    BEAM_CRITICAL("Pixel/point vectors are not of the same size.");
    return {};
  } // return nothing if not enough points
  else if (points.size() < 3) {
    BEAM_CRITICAL(
        "At least 3 pixel/point correspondences are required for p3p.");
    return {};
  }

  Eigen::Matrix4d current_pose;
  int current_inliers = 0;
  if (seed == -1) {
    srand(time(0));
  } else {
    srand(seed);
  }
  for (int epoch = 0; epoch < max_iterations; epoch++) {
    std::vector<Eigen::Vector2i> pixels_copy = pixels;
    std::vector<Eigen::Vector3d> points_copy = points;
    std::vector<Eigen::Vector2i> pixels_sample;
    std::vector<Eigen::Vector3d> points_sample;
    // fill new point vectors with randomly sampled points from inputs
    int n = pixels_copy.size();
    for (uint32_t i = 0; i < 3; i++) {
      int idx = rand() % n;
      pixels_sample.push_back(pixels_copy[idx]);
      points_sample.push_back(points_copy[idx]);
      pixels_copy.erase(pixels_copy.begin() + idx);
      points_copy.erase(points_copy.begin() + idx);
      n--;
    }

    // perform p3p on the three samples correspondences
    std::vector<Eigen::Matrix4d> poses =
        AbsolutePoseEstimator::P3PEstimator(cam, pixels_sample, points_sample);

    // check if any of the p3p solution poses have the most inliers
    for (size_t i = 0; i < poses.size(); i++) {
      int inliers = beam_cv::CheckInliers(cam, points, pixels, poses[i],
                                          inlier_threshold);
      if (inliers > current_inliers) {
        current_inliers = inliers;
        current_pose = poses[i];
      }
    }
  }
  return current_pose;
}

void AbsolutePoseEstimator::EigenWithKnownZero(const Eigen::Matrix3d& M,
                                               Eigen::Matrix3d& E, double& sig1,
                                               double& sig2) {
  // In the original paper there is a missing minus sign here (for M(0,0))
  double p1 = -M(0, 0) - M(1, 1) - M(2, 2);
  double p0 = -M(0, 1) * M(0, 1) - M(0, 2) * M(0, 2) - M(1, 2) * M(1, 2) +
              M(0, 0) * (M(1, 1) + M(2, 2)) + M(1, 1) * M(2, 2);

  double disc = std::sqrt(p1 * p1 / 4.0 - p0);
  double tmp = -p1 / 2.0;
  sig1 = tmp + disc;
  sig2 = tmp - disc;

  if (std::abs(sig1) < std::abs(sig2)) std::swap(sig1, sig2);

  double c = sig1 * sig1 + M(0, 0) * M(1, 1) - sig1 * (M(0, 0) + M(1, 1)) -
             M(0, 1) * M(0, 1);
  double a1 = (sig1 * M(0, 2) + M(0, 1) * M(1, 2) - M(0, 2) * M(1, 1)) / c;
  double a2 = (sig1 * M(1, 2) + M(0, 1) * M(0, 2) - M(0, 0) * M(1, 2)) / c;
  double n = 1.0 / std::sqrt(1 + a1 * a1 + a2 * a2);
  E.col(0) << a1 * n, a2 * n, n;

  c = sig2 * sig2 + M(0, 0) * M(1, 1) - sig2 * (M(0, 0) + M(1, 1)) -
      M(0, 1) * M(0, 1);
  a1 = (sig2 * M(0, 2) + M(0, 1) * M(1, 2) - M(0, 2) * M(1, 1)) / c;
  a2 = (sig2 * M(1, 2) + M(0, 1) * M(0, 2) - M(0, 0) * M(1, 2)) / c;
  n = 1.0 / std::sqrt(1 + a1 * a1 + a2 * a2);
  E.col(1) << a1 * n, a2 * n, n;

  E.col(2) = M.col(1).cross(M.col(2)).normalized();
}

void AbsolutePoseEstimator::RefineLambda(double& lambda1, double& lambda2,
                                         double& lambda3, const double a12,
                                         const double a13, const double a23,
                                         const double b12, const double b13,
                                         const double b23) {
  for (int iter = 0; iter < 5; ++iter) {
    double r1 = (lambda1 * lambda1 - 2.0 * lambda1 * lambda2 * b12 +
                 lambda2 * lambda2 - a12);
    double r2 = (lambda1 * lambda1 - 2.0 * lambda1 * lambda3 * b13 +
                 lambda3 * lambda3 - a13);
    double r3 = (lambda2 * lambda2 - 2.0 * lambda2 * lambda3 * b23 +
                 lambda3 * lambda3 - a23);
    if (std::abs(r1) + std::abs(r2) + std::abs(r3) < 1e-10) return;
    double x11 = lambda1 - lambda2 * b12;
    double x12 = lambda2 - lambda1 * b12;
    double x21 = lambda1 - lambda3 * b13;
    double x23 = lambda3 - lambda1 * b13;
    double x32 = lambda2 - lambda3 * b23;
    double x33 = lambda3 - lambda2 * b23;
    double detJ = 0.5 / (x11 * x23 * x32 +
                         x12 * x21 * x33); // half minus inverse determinant
    // This uses the closed form of the inverse for the jacobean.
    // Due to the zero elements this actually becomes quite nice.
    lambda1 += (-x23 * x32 * r1 - x12 * x33 * r2 + x12 * x23 * r3) * detJ;
    lambda2 += (-x21 * x33 * r1 + x11 * x33 * r2 - x11 * x23 * r3) * detJ;
    lambda3 += (x21 * x32 * r1 - x11 * x32 * r2 - x12 * x21 * r3) * detJ;
  }
}
} // namespace beam_cv