#include "beam_cv/geometry/AbsolutePoseEstimator.h"
#include "beam_cv/geometry/Triangulation.h"
#include "beam_utils/math.hpp"

#include <Eigen/Geometry>

#include <math.h>

namespace beam_cv {
std::vector<Eigen::Matrix4d> AbsolutePoseEstimator::P3PEstimator(
    std::shared_ptr<beam_calibration::CameraModel> cam,
    std::vector<Eigen::Vector2i> pixels, std::vector<Eigen::Vector3d> points) {
  // normalize pixels coords
  for (std::size_t i = 0; i < pixels.size(); ++i) { pixels[i].normalize(); };

  // compute aij and bij
  double b12 = -2.0 * pixels[1].dot(pixels[2]);
  double b13 = -2.0 * pixels[1].dot(pixels[3]);
  double b23 = -2.0 * pixels[2].dot(pixels[3]);

  Eigen::Vector3d d12 = points[1] - points[2];
  Eigen::Vector3d d13 = points[1] - points[3];
  Eigen::Vector3d d23 = points[2] - points[3];
  Eigen::Vector3d d12xd12(d12.cross(d13));

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
  double p2 = 2.0 * blob * a23 * a13 + a13 * (2.0 * a12 + a13) * s23_squared +
              a23 * (a23 - a12) * s31_squared;
  double p1 = a23 * (a13 - a23) * s12_squared - a12 * a12 * s23_squared -
              2.0 * a12 * (blob * a23 + a13 * s23_squared);
  double p0 = a12 * (a12 * s23_squared - a23 * s12_squared);

  double g = 0;
  {
    p3 = 1 / p3;
    p2 *= p3;
    p1 *= p3;
    p0 *= p3;
    g = AbsolutePoseEstimator::SolveCubic(p2, p1, p0);
  }

  double A00 = a23 * (1.0 - g);
  double A01 = (a23 * b12) * 0.5;
  double A02 = (a23 * b13 * g) * (-0.5);
  double A11 = a23 - a12 + a13 * g;
  double A12 = b23 * (a13 * g - a12) * 0.5;
  double A22 = g * (a13 - a23) - a12;

  Eigen::Matrix3d A;
  A << A00, A01, A02, A01, A11, A12, A02, A12, A22;
  Eigen::Matrix3d V;
  Eigen::Vector3d L;
  AbsolutePoseEstimator::EigenWithKnownZero(A, V, L);

  // construct D1 and D2

  // compute a real root of the cubic equation

  // D0 = D1 + gamma D2

  // solve for the two non-zero eigen values of D0

  //
  std::vector<Eigen::Matrix4d> transformations;

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

  for (unsigned int cnt = 0; cnt < iterations; ++cnt) {
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

void EigenWithKnownZero(Eigen::Matrix3d x, Eigen::Matrix3d& E,
                        Eigen::Vector3d& L) {
  Eigen::Vector3d v3(x(3) * x(7) - x(6) * x(4), x(6) * x(1) - x(7) * x(0),
                     x(4) * x(0) - x(3) * x(1));
  v3.normalize();

  double x01_squared = x(0, 1) * x(0, 1);
  // get the two other...
  double b = -x(0, 0) - x(1, 1) - x(2, 2);
  double c = -x01_squared - x(0, 2) * x(0, 2) - x(1, 2) * x(1, 2) +
             x(0, 0) * (x(1, 1) + x(2, 2)) + x(1, 1) * x(2, 2);
  double e1, e2;
  // roots(poly(x))
  AbsolutePoseEstimator::Root2Real(b, c, e1, e2);

  if (std::abs(e1) < std::abs(e2)) std::swap(e1, e2);
  L(0) = e1;
  L(1) = e2;

  double mx0011 = -x(0, 0) * x(1, 1);
  double prec_0 = x(0, 1) * x(1, 2) - x(0, 2) * x(1, 1);
  double prec_1 = x(0, 1) * x(0, 2) - x(0, 0) * x(1, 2);

  double e = e1;
  double tmp = 1.0 / (e * (x(0, 0) + x(1, 1)) + mx0011 - e * e + x01_squared);
  double a1 = -(e * x(0, 2) + prec_0) * tmp;
  double a2 = -(e * x(1, 2) + prec_1) * tmp;
  double rnorm = (1.0) / std::sqrt(a1 * a1 + a2 * a2 + 1.0);
  a1 *= rnorm;
  a2 *= rnorm;
  Eigen::Vector3d v1(a1, a2, rnorm);

  // e=e2;
  double tmp2 =
      1.0 / (e2 * (x(0, 0) + x(1, 1)) + mx0011 - e2 * e2 + x01_squared);
  double a21 = -(e2 * x(0, 2) + prec_0) * tmp2;
  double a22 = -(e2 * x(1, 2) + prec_1) * tmp2;
  double rnorm2 = 1.0 / std::sqrt(a21 * a21 + a22 * a22 + 1.0);
  a21 *= rnorm2;
  a22 *= rnorm2;
  Eigen::Vector3d v2(a21, a22, rnorm2);

  // optionally remove axb from v1,v2
  // costly and makes a very small difference!
  // v1=(v1-v1.dot(v3)*v3);v1.normalize();
  // v2=(v2-v2.dot(v3)*v3);v2.normalize();
  // v2=(v2-v1.dot(v2)*v2);v2.normalize();

  E << v1[0], v2[0], v3[0], v1[1], v2[1], v3[1], v1[2], v2[2], v3[2];
}
} // namespace beam_cv