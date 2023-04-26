/** @file
 * @ingroup utils
 *
 * Utility mathematical functions that do not fit anywhere else.
 *
 * The type definitions are shorthands for declaring `Eigen::Vector`,
 * `Eigen::Matrix` and `Eigen::Quaternion` objects. At current we assume all
 * slam code development will be using double precision floating points.
 */

#pragma once

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <unsupported/Eigen/MatrixFunctions>

#include <beam_utils/angles.h>
#include <beam_utils/time.h>

namespace beam {
/** @addtogroup utils
 *  @{ */

#ifndef BEAM_EIGEN_TYPEDEF
#  define BEAM_EIGEN_TYPEDEF

typedef Eigen::aligned_allocator<Eigen::Vector4d> AlignVec4d;
typedef Eigen::aligned_allocator<Eigen::Vector3d> AlignVec3d;
typedef Eigen::aligned_allocator<Eigen::Vector2d> AlignVec2d;
typedef Eigen::aligned_allocator<Eigen::Vector2i> AlignVec2i;
typedef Eigen::aligned_allocator<Eigen::Matrix3d> AlignMat3d;
typedef Eigen::aligned_allocator<Eigen::Matrix4d> AlignMat4d;
typedef Eigen::aligned_allocator<Eigen::Affine3d> AlignAff3d;

#endif // BEAM_EIGEN_TYPEDEF

static std::string _tmp_string{};

/**
 * Eigen vector comparator
 */
struct VecComparator {
  bool operator()(const Eigen::VectorXd& a, const Eigen::VectorXd& b) const {
    return std::lexicographical_compare(a.data(), a.data() + a.size(), b.data(),
                                        b.data() + b.size());
  }
};

/**
 * Eigen matrix comparator
 */
struct MatComparator {
  bool operator()(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) const {
    return std::lexicographical_compare(a.data(), a.data() + a.size(), b.data(),
                                        b.data() + b.size());
  }
};

/** Generates random integer with a upper bound `ub` and lower bound `lb` using
 * a uniform random distribution. */
int randi(int ub, int lb);

/** Generates random float with a upper bound `ub` and lower bound `lb` using a
 * uniform random distribution. */
double randf(double ub, double lb);

/** Compares two floating point numbers `f1` and `f2` with an error
 * threshold defined by `threshold`.
 * @return
 * - `0`: If `f1` == `f2` or the difference is less then `threshold`
 * - `1`: If `f1` > `f2`
 * - `-1`: If `f1` < `f2`
 */
int fltcmp(double f1, double f2, double threshold = 0.0001);

/** @return the median of `v`. */
double median(std::vector<double> v);

template <typename T, int N>
T distance(const Eigen::Matrix<T, N, 1>& lhs,
           const Eigen::Matrix<T, N, 1>& rhs) {
  T dist = 0;
  for (size_t i = 0; i < N; ++i) {
    dist += (lhs[i] - rhs[i]) * (lhs[i] - rhs[i]);
  }
  return sqrt(dist);
}
template <typename P, typename P2>
double distance(const P& lhs, const P2& rhs) {
  double dist = 0;
  for (size_t i = 0; i < 3; ++i) {
    dist += (lhs.data[i] - rhs.data[i]) * (lhs.data[i] - rhs.data[i]);
  }
  return sqrt(dist);
}

template <typename P, typename T, int N>
T distance(const P& lhs, const Eigen::Matrix<T, N, 1>& rhs) {
  T dist = 0;
  for (size_t i = 0; i < N; ++i) {
    dist += (lhs.data[i] - rhs[i]) * (lhs.data[i] - rhs[i]);
  }
  return sqrt(dist);
}

template <typename T>
std::vector<T> RandomSample(const std::vector<T>& input, uint32_t N, int seed) {
  srand(seed);
  std::vector<T> input_copy = input;
  // fill new point vectors with randomly sampled points from xs and xss
  std::vector<T> sampled;
  int n = input_copy.size();
  for (uint32_t i = 0; i < N; i++) {
    int idx = rand() % n;
    sampled.push_back(input_copy[idx]);
    input_copy.erase(input_copy.begin() + idx);
    n--;
  }
  return sampled;
}

/** Computes greatest common divisor**/
int gcd(int a, int b);

/** Reshapes a vector `x` to matrix `y` of size `rows` and `cols` */
void vec2mat(const std::vector<double>& x, int rows, int cols,
             Eigen::MatrixXd& y);

/** Reshapes a matrix to a vector*/
void mat2vec(const Eigen::MatrixXd& A, std::vector<double>& x);

/** Reshapes a vector `x` to matrix `y` of size `rows` and `cols` */
void vec2mat(const Eigen::VectorXd& x, int rows, int cols, Eigen::MatrixXd& y);

/** Reshapes a matrix to a vector*/
void mat2vec(const Eigen::MatrixXd& A, Eigen::VectorXd& x);

/** ENU to NWU coordinate system **/
void enu2nwu(const Eigen::Vector3d& enu, Eigen::Vector3d& nwu);

/** NED to ENU coordinate system **/
void ned2enu(const Eigen::Vector3d& ned, Eigen::Vector3d& enu);

/** NED to NWU coordinate system **/
void ned2nwu(const Eigen::Quaterniond& ned, Eigen::Quaterniond& enu);

/** NWU to ENU coordinate system **/
void nwu2enu(const Eigen::Vector3d& nwu, Eigen::Vector3d& enu);

/** NWU to NED coordinate system **/
void nwu2ned(const Eigen::Quaterniond& nwu, Eigen::Quaterniond& ned);

/** NWU to EDN coordinate system **/
void nwu2edn(const Eigen::Vector3d& nwu, Eigen::Vector3d& edn);

/**
 * @brief Round a matrix values to a certain precision.
 * @param precision = 100 would round to the second decimal point (i.e. 1.126
 *= 1.13)
 **/
Eigen::MatrixXd RoundMatrix(const Eigen::MatrixXd& M, int precision);

/**
 * @brief Computes kronecker product in brute force manner
 * @param A n x m input matrix
 * @param B p x q input matrix
 * @return pm x qn matrix
 **/
Eigen::MatrixXd KroneckerProduct(const Eigen::MatrixXd& A,
                                 const Eigen::MatrixXd& B);

/**
 * @brief Computes the tangential basis for a direciton vector in 3 space
 * @param x 3d vector
 * @return 3x2 matrix
 **/
Eigen::Matrix<double, 3, 2> S2TangentialBasis(const Eigen::Vector3d& x);

/**
 * @brief Linear interpolation of a vector
 * @param v1 first vector
 * @param v2 second vector
 * @param t1 time point of first vector
 * @param t2 time point of second vector
 * @param t time point that you want to interpolate at
 * @return interpolated vector
 **/
Eigen::VectorXd InterpolateVector(const Eigen::VectorXd& v1, const double& t1,
                                  const Eigen::VectorXd& v2, const double& t2,
                                  const double& t);

/**
 * @brief Fits plane to set of points
 * @return <centroid,normal>
 * @param vector of points
 */
std::pair<Eigen::Vector3d, Eigen::Vector3d>
    FitPlane(const std::vector<Eigen::Vector3d>& c);

/**
 * @brief Computes intersection point of line and plane
 * @return {x,y,z}
 */
Eigen::Vector3d IntersectPoint(const Eigen::Vector3d& ray_vector,
                               const Eigen::Vector3d& ray_point,
                               const Eigen::Vector3d& plane_normal,
                               const Eigen::Vector3d& plane_point);

/**
 * @brief return log odds of a probability
 * @param p probability
 * @return l
 */
double Logit(double p);

/**
 * @brief returns the inverse of the log odds probability function
 * @param l log odds
 * @return probability
 */
double LogitInv(double l);

/**
 * @brief applies a Bayesian Log Odds update
 * @param pk probability associated with each measurement
 * @param l0 log odds of the original probability (usually Logit(0.5) if
 * uncertain)
 * @param p_prev previous probability
 * @return updated probability
 */
double BayesianLogitUpdate(double pk, double l0, double p_prev);

/// @brief
/// @param mean
/// @param stddev
/// @return
double GaussianRandomNumber(double mean, double stddev) {
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<> dist{mean, stddev};
  return dist(gen);
}

/// @brief
/// @tparam size
/// @param mean
/// @param stddev
/// @return
template <int size>
Eigen::Vector<double, size, 1> GaussianRandomVector(double mean,
                                                    double stddev) {
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<> dist{mean, stddev};
  Eigen::Vector<double, size, 1> v;
  for (int i = 0; i < size; i++) { v(i) = dist(gen); }
  return v;
}

/// @brief
/// @param R
/// @return
Eigen::Vector3d RotationMatrixToCompactQuaternion(const Eigen::Matrix3d& R) {
  Eigen::Quaterniond q(R);
  q.normalize();
  if (q.w() < 0) { return Eigen::Vector3d(-q.vec()); };
  return q.vec();
}

/// @brief
/// @param q
/// @return
Eigen::Matrix3d CompactQuaternionToRotationMatrix(const Eigen::Vector3d& q) {
  double w = 1.0 - q.squaredNorm();
  asser(w >= 0);
  w = std::sqrt(w);
  return Eigen::Quaterniond(w, q(1), q(2), q(3)).toRotationMatrix();
}

/// @brief Convert an SE(3) transform into a twist vector {tx, ty, tz, qx, qy,
/// qz}, A compact quaternion is the imaginary part of a quaternion, the real
/// part can be recovered with 1 - q.squaredNorm()
/// @param T input transform
/// @return twist vector representation of transform
Eigen::Vector6d TransformToTwistVector(const Eigen::Matrix4d& T) {
  assert(beam::IsTransformationMatrix(T));
  const auto linear = T.block<3, 3>(0, 0);
  const auto translation = T.block<3, 1>(0, 3);

  Eigen::Vector6d v;
  v.block<3, 1>(0, 0) = translation;
  v.block<3, 1>(3, 0) = RotationMatrixToCompactQuaternion(linear);
  return v;
}

/// @brief
/// @param v
/// @return
Eigen::Matrix4d TwistVectorToTransform(const Eigen::Vector6d& v) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = CompactQuaternionToRotationMatrix(v.tail<3>());
  T.block<3, 1>(0, 3) = v.head<3>().transpose();
  return T;
}

/// @brief Computes the difference between two poses as a twist vector
/// @param T1
/// @param T2
/// @return
Eigen::Vector6d BoxMinus(const Eigen::Matrix4d& T1, const Eigen::Matrix4d& T2) {
  return TransformToTwistVector(beam::InvertTransform(T1) * T2);
}

/// @brief Perturbs a pose with parameters (given as a twist vector)
/// @param T1
/// @param perturbation
/// @return
Eigen::Matrix4d BoxPlus(const Eigen::Matrix4d& T,
                        const Vector6d& perturbation) {
  return T * TwistVectorToTransform(perturbation);
}

/** @} group utils */
} // namespace beam
