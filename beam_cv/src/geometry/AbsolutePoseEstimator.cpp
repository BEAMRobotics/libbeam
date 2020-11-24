#include "beam_cv/geometry/AbsolutePoseEstimator.h"
#include "beam_cv/geometry/Triangulation.h"
#include "beam_utils/math.hpp"

#include <Eigen/Geometry>

#include <math.h>

namespace beam_cv {
std::vector<Eigen::Matrix4d> AbsolutePoseEstimator::P3PEstimator(
    std::shared_ptr<beam_calibration::CameraModel> cam,
    std::vector<Eigen::Vector2i> pixels, std::vector<Eigen::Vector3d> points,
    int cubic_iterations, int refinement_iterations) {
  std::vector<Eigen::Vector3d> y;
  // store normalized pixels coords
  for (std::size_t i = 0; i < pixels.size(); ++i) {
    Eigen::Vector3d homogenized_pixel = cam->BackProject(pixels[i]).value();
    y.push_back(homogenized_pixel);
  };

  // compute aij and bij
  double b12 = -2 * (y[0].dot(y[1]));
  double b13 = -2 * (y[0].dot(y[2]));
  double b23 = -2 * (y[1].dot(y[2]));

  Eigen::Vector3d d12 = points[0] - points[1];
  Eigen::Vector3d d13 = points[0] - points[2];
  Eigen::Vector3d d23 = points[1] - points[2];
  Eigen::Vector3d d12xd13(d12.cross(d13));

  double a12 = d12.squaredNorm();
  double a13 = d13.squaredNorm();
  double a23 = d23.squaredNorm();

  double c31 = -.5 * b13;
  double c23 = -.5 * b23;
  double c12 = -.5 * b12;
  double blob = c12 * c23 * c31 - 1;

  double s31_squared = 1 - c31 * c31;
  double s23_squared = 1 - c23 * c23;
  double s12_squared = 1 - c12 * c12;

  double p3 = (a13 * (a23 * s31_squared - a13 * s23_squared));
  double p2 = 2 * blob * a23 * a13 + a13 * (2 * a12 + a13) * s23_squared +
              a23 * (a23 - a12) * s31_squared;
  double p1 = a23 * (a13 - a23) * s12_squared - a12 * a12 * s23_squared -
              2 * a12 * (blob * a23 + a13 * s23_squared);
  double p0 = a12 * (a12 * s23_squared - a23 * s12_squared);

  double g = 0;
  {
    p3 = 1 / p3;
    p2 *= p3;
    p1 *= p3;
    p0 *= p3;
    g = AbsolutePoseEstimator::SolveCubic(p2, p1, p0, cubic_iterations);
  }

  double A00 = a23 * (1 - g);
  double A01 = (a23 * b12) * .5;
  double A02 = (a23 * b13 * g) * (-.5);
  double A11 = a23 - a12 + a13 * g;
  double A12 = b23 * (a13 * g - a12) * .5;
  double A22 = g * (a13 - a23) - a12;

  Eigen::Matrix3d A;
  A << A00, A01, A02, A01, A11, A12, A02, A12, A22;
  Eigen::Matrix3d V;
  Eigen::Vector3d L;
  AbsolutePoseEstimator::EigenWithKnownZero(A, V, L);
  double v = std::sqrt(std::max(double(0), -L(1) / L(0)));
  std::cout << V << std::endl;
  std::cout << L << std::endl;
  std::cout << v << std::endl;

  // find lambda vectors representing valid solutions
  std::vector<Eigen::Vector3d> Ls;

  { // valid solutions via positive v
    double s = v;

    double w2 = 1.0 / (s * V(1) - V(0));
    double w0 = (V(3) - s * V(4)) * w2;
    double w1 = (V(6) - s * V(7)) * w2;

    double a = 1.0 / ((a13 - a12) * w1 * w1 - a12 * b13 * w1 - a12);
    double b =
        (a13 * b12 * w1 - a12 * b13 * w0 - 2.0 * w0 * w1 * (a12 - a13)) * a;
    double c = ((a13 - a12) * w0 * w0 + a13 * b12 * w0 + a13) * a;

    if (b * b - 4.0 * c >= 0) {
      double tau1, tau2;
      AbsolutePoseEstimator::Root2Real(b, c, tau1, tau2);
      std::cout << std::endl << "+v tau1: " << tau1 << std::endl;
      std::cout << std::endl << "+v tau2: " << tau2 << std::endl;
      if (tau1 > 0) {
        double tau = tau1;
        double d = a23 / (tau * (b23 + tau) + 1.0);

        double l2 = std::sqrt(d);
        double l3 = tau * l2;

        double l1 = w0 * l2 + w1 * l3;
        if (l1 >= 0) { Ls.push_back(Eigen::Vector3d(l1, l2, l3)); }
      }
      if (tau2 > 0) {
        double tau = tau2;
        double d = a23 / (tau * (b23 + tau) + 1.0);

        double l2 = std::sqrt(d);
        double l3 = tau * l2;
        double l1 = w0 * l2 + w1 * l3;
        if (l1 >= 0) { Ls.push_back(Eigen::Vector3d(l1, l2, l3)); }
      }
    }
  }

  { // valid solutions via negative v
    double s = -v;
    double w2 = 1.0 / (s * V(0, 1) - V(0, 0));
    double w0 = (V(1, 0) - s * V(1, 1)) * w2;
    double w1 = (V(2, 0) - s * V(2, 1)) * w2;

    double a = 1.0 / ((a13 - a12) * w1 * w1 - a12 * b13 * w1 - a12);
    double b =
        (a13 * b12 * w1 - a12 * b13 * w0 - 2.0 * w0 * w1 * (a12 - a13)) * a;
    double c = ((a13 - a12) * w0 * w0 + a13 * b12 * w0 + a13) * a;

    if (b * b - 4.0 * c >= 0) {
      double tau1, tau2;
      Root2Real(b, c, tau1, tau2);
      std::cout << std::endl << "-v tau1: " << tau1 << std::endl;
      std::cout << std::endl << "-v tau2: " << tau2 << std::endl;
      if (tau1 > 0) {
        double tau = tau1;
        double d = a23 / (tau * (b23 + tau) + 1.0);
        if (d > 0) {
          double l2 = std::sqrt(d);

          double l3 = tau * l2;

          double l1 = w0 * l2 + w1 * l3;
          if (l1 >= 0) { Ls.push_back(Eigen::Vector3d(l1, l2, l3)); }
        }
      }
      if (tau2 > 0) {
        double tau = tau2;
        double d = a23 / (tau * (b23 + tau) + 1.0);
        if (d > 0) {
          double l2 = std::sqrt(d);

          double l3 = tau * l2;

          double l1 = w0 * l2 + w1 * l3;
          if (l1 >= 0) { Ls.push_back(Eigen::Vector3d(l1, l2, l3)); }
        }
      }
    }
  }

  std::cout << std::endl << "Ls final size: " << Ls.size() << std::endl;

  for (size_t i = 0; i < Ls.size(); ++i) {
    AbsolutePoseEstimator::GaussNewtonRefine(Ls[i], a12, a13, a23, b12, b13,
                                             b23, refinement_iterations);
  }

  std::vector<Eigen::Matrix4d> transformations;

  Eigen::Vector3d ry1, ry2, ry3;
  Eigen::Vector3d yd1;
  Eigen::Vector3d yd2;
  Eigen::Vector3d yd1xd2;
  Eigen::Matrix3d X;
  X << d12(0), d13(0), d12xd13(0), d12(1), d13(1), d12xd13(1), d12(2), d13(2),
      d12xd13(2);
  X = X.inverse();

  for (size_t i = 0; i < Ls.size(); ++i) {
    // cout<<"Li="<<Ls(i)<<endl;

    // compute the rotation:
    ry1 = y[0] * Ls[i](0);
    ry2 = y[1] * Ls[i](1);
    ry3 = y[2] * Ls[i](2);

    yd1 = ry1 - ry2;
    yd2 = ry1 - ry3;
    yd1xd2 = yd1.cross(yd2);

    Eigen::Matrix3d Y;
    Y << yd1(0), yd2(0), yd1xd2(0), yd1(1), yd2(1), yd1xd2(1), yd1(2), yd2(2),
        yd1xd2(2);

    Eigen::Matrix4d solution = Eigen::Matrix4d::Zero();
    solution(3, 3) = 1;
    // add rotation to top-left 3x3
    solution.block(0, 0, 3, 3) = Y * X;
    // add translation to right column
    // solution.col(3) = (ry1 - solution.block(0, 0, 3, 3) * points[0]);
    std::cout << solution << std::endl << points[0];
    transformations.push_back(solution);
  }

  return transformations;
}

// adapted from lambdatwist github

/**
 * h(r) = r^3 + b*r^2 + c*r + d = 0
 *
 * The return root is as stable as possible in the sense that it has as high
 * derivative as possible.  The solution is found by simple Newton-Raphson
 * iterations, and the trick is to choose the intial solution r0 in a clever
 * way.
 *
 * The intial solution is found by considering 5 cases:
 *
 * Cases I and II: h has no stationary points. In this case its derivative
 * is positive.  The inital solution to the NR-iteration is r0 here h has
 * minimal derivative.
 *
 * Case III, IV, and V: has two stationary points, t1 < t2.  In this case,
 * h has negative derivative between t1 and t2.  In these cases, we can make
 * a second order approximation of h around each of t1 and t2, and choose r0
 * as the leftmost or rightmost root of these approximations, depending on
 * whether two, one, or both of h(t1) and h(t2) are > 0.
 */
double AbsolutePoseEstimator::SolveCubic(double b, double c, double d,
                                         int iterations) {
  /* Choose initial solution */
  double r0;
  // not monotonic
  if (b * b >= 3.0 * c) {
    // h has two stationary points, compute them
    // T t1 = t - std::sqrt(diff);
    double v = std::sqrt(b * b - 3.0 * c);
    double t1 = (-b - v) / (3.0);

    // Check if h(t1) > 0, in this case make a 2-order approx of h around t1
    double k = ((t1 + b) * t1 + c) * t1 + d;

    if (k > 0.0) {
      // Find leftmost root of 0.5*(r0 -t1)^2*(6*t1+2*b) +  k = 0
      r0 = t1 - std::sqrt(-k / (3.0 * t1 + b));
      // or use the linear comp too
      // r0=t1 -
    } else {
      double t2 = (-b + v) / (3.0);
      k = ((t2 + b) * t2 + c) * t2 + d;
      // Find rightmost root of 0.5*(r0 -t2)^2*(6*t2+2*b) +  k1 = 0
      r0 = t2 + std::sqrt(-k / (3.0 * t2 + b));
    }
  } else {
    r0 = -b / 3.0;
    if (std::abs(((3.0 * r0 + 2.0 * b) * r0 + c)) < 1e-4) r0 += 1;
  }

  /* Do ITER Newton-Raphson iterations */
  /* Break if position of root changes less than 1e.16 */
  // T starterr=std::abs(r0*(r0*(r0 + b) + c) + d);
  double fx, fpx;

  for (int cnt = 0; cnt < iterations; ++cnt) {
    //(+ (* r0 (+  c (* (+ r0 b) r0) )) d )
    fx = (((r0 + b) * r0 + c) * r0 + d);

    // 1e-13 is the numeric limit of double
    if ((cnt < 7 || std::abs(fx) > 1e-13)) {
      fpx = ((3.0 * r0 + 2.0 * b) * r0 + c);

      r0 -= fx / fpx;
    } else
      break;
  }

  return r0;
}

bool AbsolutePoseEstimator::Root2Real(double b, double c, double& r1,
                                      double& r2) {
  double v = b * b - 4.0 * c;
  if (v < 0) {
    r1 = r2 = 0.5 * b;
    return false;
  }

  double y = std::sqrt(v);
  if (b < 0) {
    r1 = 0.5 * (-b + y);
    r2 = 0.5 * (-b - y);
  } else {
    r1 = 2.0 * c / (-b + y);
    r2 = 2.0 * c / (-b - y);
  }
  return true;
}

void AbsolutePoseEstimator::EigenWithKnownZero(Eigen::Matrix3d x,
                                               Eigen::Matrix3d E,
                                               Eigen::Vector3d L) {
  double p1 = -x(0, 0) - x(1, 1) - x(2, 2);
  double p0 = -x(0, 1) * x(0, 1) - x(0, 2) * x(0, 2) - x(1, 2) * x(1, 2) +
              x(0, 0) * (x(1, 1) + x(2, 2)) + x(1, 1) * x(2, 2);

  double disc = std::sqrt(p1 * p1 / 4.0 - p0);
  double tmp = -p1 / 2.0;
  double sig1 = tmp + disc;
  double sig2 = tmp - disc;

  if (std::abs(sig1) < std::abs(sig2)) std::swap(sig1, sig2);
  L(0) = sig1;
  L(1) = sig2;
  L(2) = 0;

  double c = sig1 * sig1 + x(0, 0) * x(1, 1) - sig1 * (x(0, 0) + x(1, 1)) -
             x(0, 1) * x(0, 1);
  double a1 = (sig1 * x(0, 2) + x(0, 1) * x(1, 2) - x(0, 2) * x(1, 1)) / c;
  double a2 = (sig1 * x(1, 2) + x(0, 1) * x(0, 2) - x(0, 0) * x(1, 2)) / c;
  double n = 1.0 / std::sqrt(1 + a1 * a1 + a2 * a2);
  E.col(0) << a1 * n, a2 * n, n;

  c = sig2 * sig2 + x(0, 0) * x(1, 1) - sig2 * (x(0, 0) + x(1, 1)) -
      x(0, 1) * x(0, 1);
  a1 = (sig2 * x(0, 2) + x(0, 1) * x(1, 2) - x(0, 2) * x(1, 1)) / c;
  a2 = (sig2 * x(1, 2) + x(0, 1) * x(0, 2) - x(0, 0) * x(1, 2)) / c;
  n = 1.0 / std::sqrt(1 + a1 * a1 + a2 * a2);
  E.col(1) << a1 * n, a2 * n, n;

  E.col(2) = x.col(1).cross(x.col(2)).normalized();
}

void AbsolutePoseEstimator::GaussNewtonRefine(Eigen::Vector3d& L, double a12,
                                              double a13, double a23,
                                              double b12, double b13,
                                              double b23, int iterations) {
  for (int i = 0; i < iterations; ++i) {
    double l1 = L(0);
    double l2 = L(1);
    double l3 = L(2);
    double r1 = l1 * l1 + l2 * l2 + b12 * l1 * l2 - a12;
    double r2 = l1 * l1 + l3 * l3 + b13 * l1 * l3 - a13;
    double r3 = l2 * l2 + l3 * l3 + b23 * l2 * l3 - a23;

    if (std::abs(r1) + std::abs(r2) + std::abs(r3) < 1e-10) break;

    double dr1dl1 = (2.0) * l1 + b12 * l2;
    double dr1dl2 = (2.0) * l2 + b12 * l1;
    // T dr1dl3=0;

    double dr2dl1 = (2.0) * l1 + b13 * l3;
    // T dr2dl2=0;
    double dr2dl3 = (2.0) * l3 + b13 * l1;

    // T dr3dl1=0;
    double dr3dl2 = (2.0) * l2 + b23 * l3;
    double dr3dl3 = (2.0) * l3 + b23 * l2;

    Eigen::Vector3d r(r1, r2, r3);

    {
      double v0 = dr1dl1;
      double v1 = dr1dl2;
      double v3 = dr2dl1;
      double v5 = dr2dl3;
      double v7 = dr3dl2;
      double v8 = dr3dl3;
      double det = (1.0) / (-v0 * v5 * v7 - v1 * v3 * v8);

      Eigen::Matrix3d Ji;
      Ji << -v5 * v7, -v1 * v8, v1 * v5, -v3 * v8, v0 * v8, -v0 * v5, v3 * v7,
          -v0 * v7, -v1 * v3;
      Eigen::Vector3d L1 = Eigen::Vector3d(L) - det * (Ji * r);

      {
        double l1 = L1(0);
        double l2 = L1(1);
        double l3 = L1(2);
        double r11 = l1 * l1 + l2 * l2 + b12 * l1 * l2 - a12;
        double r12 = l1 * l1 + l3 * l3 + b13 * l1 * l3 - a13;
        double r13 = l2 * l2 + l3 * l3 + b23 * l2 * l3 - a23;
        if (std::abs(r11) + std::abs(r12) + std::abs(r13) >
            std::abs(r1) + std::abs(r2) + std::abs(r3)) {
          break;
        } else
          L = L1;
      }
    }
  }
}
} // namespace beam_cv