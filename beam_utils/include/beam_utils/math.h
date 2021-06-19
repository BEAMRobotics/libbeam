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
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <unsupported/Eigen/MatrixFunctions>

#include <beam_utils/angles.h>
#include <beam_utils/time.h>

namespace beam {
/** @addtogroup utils
 *  @{ */

#ifndef BEAM_EIGEN_TYPEDEF
#  define BEAM_EIGEN_TYPEDEF
typedef Eigen::Vector2d Vec2;
typedef Eigen::Vector3d Vec3;
typedef Eigen::Vector4d Vec4;
typedef Eigen::Matrix<double, 5, 1> Vec5;
typedef Eigen::Matrix<double, 6, 1> Vec6;
typedef Eigen::VectorXd VecX;

typedef Eigen::Matrix2d Mat2;
typedef Eigen::Matrix3d Mat3;
typedef Eigen::Matrix4d Mat4;
typedef Eigen::Matrix<double, 5, 5> Mat5;
typedef Eigen::Matrix<double, 6, 6> Mat6;
typedef Eigen::Matrix<double, 7, 7> Mat7;
typedef Eigen::Matrix<double, 9, 9> Mat9;
typedef Eigen::MatrixXd MatX;

typedef Eigen::Affine3d Affine3;

typedef Eigen::Quaterniond Quaternion;
#endif // BEAM_EIGEN_TYPEDEF

/**
 * Eigen vector comparator
 */
struct VecComparator {
  bool operator()(const VecX& a, const VecX& b) const {
    return std::lexicographical_compare(a.data(), a.data() + a.size(), b.data(),
                                        b.data() + b.size());
  }
};

/**
 * Eigen matrix comparator
 */
struct MatComparator {
  bool operator()(const MatX& a, const MatX& b) const {
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

/** Compares two floating point numbers `f1` and `f2` with an error threshold
 * defined by `threshold`.
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
std::vector<T> RandomSample(std::vector<T>& input, uint32_t N, int seed) {
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
void vec2mat(std::vector<double> x, int rows, int cols, MatX& y);

/** Reshapes a matrix to a vector*/
void mat2vec(MatX A, std::vector<double>& x);

/** Reshapes a vector `x` to matrix `y` of size `rows` and `cols` */
void vec2mat(VecX x, int rows, int cols, MatX& y);

/** Reshapes a matrix to a vector*/
void mat2vec(MatX A, VecX& x);

/** Convert euler angle to rotation matrix **/
int euler2rot(Vec3 euler, int euler_seq, Mat3& R);

/** Convert euler angle to quaternion **/
int euler2quat(Vec3 euler, int euler_seq, Quaternion& q);

/** Convert quaternion to euler angles **/
int quat2euler(Quaternion q, int euler_seq, Vec3& euler);

/** Convert quaternion to rotation matrix **/
int quat2rot(Quaternion q, Mat3& R);

/** ENU to NWU coordinate system **/
void enu2nwu(const Vec3& enu, Vec3& nwu);

/** NED to ENU coordinate system **/
void ned2enu(const Vec3& ned, Vec3& enu);

/** NED to NWU coordinate system **/
void ned2nwu(const Quaternion& ned, Quaternion& enu);

/** NWU to ENU coordinate system **/
void nwu2enu(const Vec3& nwu, Vec3& enu);

/** NWU to NED coordinate system **/
void nwu2ned(const Quaternion& nwu, Quaternion& ned);

/** NWU to EDN coordinate system **/
void nwu2edn(const Vec3& nwu, Vec3& edn);

/**
 * @brief Computes the right jacobian of a 3-vector representing the exponential
 * coordinates in SO(3).
 * 
 * The right jacobian of a 3-vector in Lie space is computed as:
 * I - (1-cos(||w||))/(||w||^2)*[w]_x + (||w|| - sin(||w||))/(||w||^3)*[w]_x
 *
 * Jacobians on Lie groups are explained in detail on page 8 in:
 *https://arxiv.org/pdf/1812.01537.pdf
 * @param w vector to compute right jacobian of
 **/
Eigen::Matrix3d RightJacobianOfSO3(const Eigen::Vector3d& w);

/**
 * @brief Round a matrix values to a certain precision.
 * @param precision = 100 would round to the second decimal point (i.e. 1.126
 *= 1.13)
 **/
MatX RoundMatrix(const MatX& M, int precision);

/**
 * @brief check if a matrix is a valid transformation matrix
 * @param T tranformation
 **/
bool IsTransformationMatrix(Eigen::Matrix4d T);

/**
 * @brief check if a matrix is a valid rotation matrix
 * @param R rotation matrix
 **/
bool IsRotationMatrix(Eigen::Matrix3d R);

/**
 * @brief Computes kronecker product in brute force manner
 * @param A n x m input matrix
 * @param B p x q input matrix
 * @return pm x qn matrix
 **/
Eigen::MatrixXd KroneckerProduct(Eigen::MatrixXd A, Eigen::MatrixXd B);

/**
 * @brief Convert from rotation matrix to its associated Lie Algebra
 * @param R rotation matrix
 * @return 3x1 vector representing R in Lie Algebra space
 **/
beam::Vec3 RToLieAlgebra(const beam::Mat3 R);

/**
 * @brief Convert from Lie Algebra to its associated rotation matrix
 * @param eps rotation in Lie Algebra space
 * @return rotation matrix
 **/
beam::Mat3 LieAlgebraToR(const beam::Vec3 eps);

/**
 * @brief Linear interpolation of transformations using a method in Tim
 *Barfoot's State Estimation textbook
 * @param m1 first transformation matrix
 * @param m2 second transformation matrix
 * @param t1 time point of first transform
 * @param t2 time point of second transform
 * @param t time point that you want to interpolate at
 * @return interpolated transformation matrix
 **/
beam::Mat4 InterpolateTransform(const beam::Mat4& m1, const beam::TimePoint& t1,
                                const beam::Mat4& m2, const beam::TimePoint& t2,
                                const beam::TimePoint& t);

/**
 * @brief Perform inverse of skew symmetric transform
 * @param M 3x3 skew symmetric matrix
 * @return 3x1 vector
 **/
beam::Vec3 InvSkewTransform(const beam::Mat3 M);

/**
 * @brief Perform skew symmetric transform
 * @param V 3x1 vector
 * @return 3x3 skew symmetric matrix
 **/
beam::Mat3 SkewTransform(const beam::Vec3 V);

/**
 * @brief Inverts a 4x4 Transformation matrix by taking into account that
 *R^(-1)=R^T
 * @param T Transform matrix to be inverted
 * @return 4x4 inverted transformation matrix
 **/
Eigen::Matrix4d InvertTransform(const Eigen::MatrixXd& T);

/**
 * @brief Fits plane to set of points
 * @return <centroid,normal>
 * @param vector of points
 */
std::pair<beam::Vec3, beam::Vec3> FitPlane(const std::vector<beam::Vec3>& c);

/**
 * @brief Computes intersection point of line and plane
 * @return {x,y,z}
 */
beam::Vec3 IntersectPoint(beam::Vec3 ray_vector, beam::Vec3 ray_point,
                          beam::Vec3 plane_normal, beam::Vec3 plane_point);

/** Peturbs a transformation
 * @param[in] T_in the original transformation matrix
 * @param[in] perturbations [rx(rad), ry(rad), rz(rad), tx(m), ty(m),
 * tx(m)]
 * @return perturbed transformation
 */
Eigen::Matrix4d PerturbTransformRadM(const Eigen::Matrix4d& T_in,
                                     const Eigen::VectorXd& perturbations);

/** Peturbs a transformation
 * @param[in] T_in the original transformation matrix
 * @param[in] perturbations [rx(deg), ry(deg), rz(deg), tx(m), ty(m),
 * tx(m)]
 * @return perturbed transformation
 */
Eigen::Matrix4d PerturbTransformDegM(const Eigen::Matrix4d& T_in,
                                     const Eigen::VectorXd& perturbations);

/**
 * @brief build a 4x4 transformation matrix from rpy (in degrees) and
 * translation (in m)
 * @param rollInDeg rotation about x
 * @param pitchInDeg rotation about y
 * @param yawInDeg rotation about z
 * @param tx translation in x
 * @param ty translation in y
 * @param tz translation in z
 * @return transformation matrix [R t ; 0 1]
 */
Eigen::Matrix4d BuildTransformEulerDegM(double rollInDeg, double pitchInDeg,
                                        double yawInDeg, double tx, double ty,
                                        double tz);

/**
 * @brief convert vector of quaternion and translation [qw
 * qx qy qz tx ty tx] to eigen matrix pose. This vector form is usually the
 * format used in Ceres when using the quaternion + identity parameterization
 * @param pose [qw qx qy qz tx ty tx]
 */
Eigen::Matrix4d
    QuaternionAndTranslationToTransformMatrix(const std::vector<double>& pose);

/**
 * @brief convert eigen matrix pose to vector of quaternion and translation [qw
 * qx qy qz tx ty tx]. This is the format used in Ceres, simply build this pose
 * and send the &pose to Ceres with a quaternion + identity parameterization
 * @param T pose matrix
 */
std::vector<double>
    TransformMatrixToQuaternionAndTranslation(const Eigen::Matrix4d& T);

/**
 * @brief convert vector of quaternion and translation [qw
 * qx qy qz tx ty tx] to eigen matrix pose. This vector form is usually the
 * format used in Ceres when using the quaternion + identity parameterization
 * @param q input quaternion
 * @param p input pose
 * @param T output Transformation matrix
 */
void QuaternionAndTranslationToTransformMatrix(const Eigen::Quaterniond& q,
                                               const Eigen::Vector3d& p,
                                               Eigen::Matrix4d& T);
/**
 * @brief convert eigen matrix pose to vector of quaternion and translation [qw
 * qx qy qz tx ty tx]. This is the format used in Ceres, simply build this pose
 * and send the &pose to Ceres with a quaternion + identity parameterization
 * @param T input Transformation matrix
 * @param q output quaternion
 * @param p output pose
 */
void TransformMatrixToQuaternionAndTranslation(const Eigen::Matrix4d& T,
                                               Eigen::Quaterniond& q,
                                               Eigen::Vector3d& p);

/**
 * @brief Determine if two poses (in matrix form) differ by some motion
 * threshold, parameterized by a rotational and translational
 * error. This works by first taking inv(T1) * T2 which gets the relative
 * transform between the two poses. Then it calculated the norm of the
 * translation, and the absolute value of the angle in AxisAngle representation
 *
 * Two main use cases:
 *  (1) Checking if poses are equal:
 *      (a) set is_threshold_minimum to false
 *      (b) set must_pass_both to true
 *      (c) set thresholds to something small based on your desired accuracy
 *  (2) Checking for minimal change in pose (e.g., to know if we want to extract
 * a new keyframe) (c) set is_threshold_minimum to true (b) set must_pass_both
 * to false (c) pick motion thresholds
 *
 * @param T1 pose1
 * @param T2 pose2
 * @param angle_threshold_deg angle threshold in degrees
 * @param translation_threshold_m translation threshold in meters
 * @param is_threshold_minimum requires motion to be greater than threshold if
 * set to true
 * @param must_pass_both if set, it will return true only if both angle and
 * translation thresholds are passed
 * @param output_error_cause if set to true and this function returns false, it
 * will output the cause of the error to cout
 * @return result
 */
bool PassedMotionThreshold(const Eigen::Matrix4d& T1, const Eigen::Matrix4d& T2,
                           double angle_threshold_deg,
                           double translation_threshold_m,
                           bool is_threshold_minimum = true,
                           bool must_pass_both = false,
                           bool output_error_cause = false);

/**
 * @brief Wrapper around PassedMotionThreshold that sets is_threshold_minimum to
 * false and must_pass_both to true. This can be used to check if two estimates
 * of the same pose are similar
 * @param T1 pose1
 * @param T2 pose2
 * @param angle_threshold_deg maximum angle error in degrees
 * @param translation_threshold_m maximum translation error in meters
 * @param output_error_cause if set to true and this function returns false, it
 * will output the cause of the error to cout
 * @return result
 */
bool ArePosesEqual(const Eigen::Matrix4d& T1, const Eigen::Matrix4d& T2,
                   double angle_threshold_deg = 1,
                   double translation_threshold_m = 0.005,
                   bool output_error_cause = false);

/**
 * @brief Wrapper around PassedMotionThreshold that sets is_threshold_minimum to
 * true and must_pass_both to false. This can be used to check if two poses
 * differ by some rotation OR some translation.
 * @param T1 pose1
 * @param T2 pose2
 * @param angle_threshold_deg minimum angle change in degrees
 * @param translation_threshold_m minimum translation change in meters
 * @param output_error_cause if set to true and this function returns false, it
 * will output the cause of the error to cout
 * @return result
 */
bool PassedMinMotion(const Eigen::Matrix4d& T1, const Eigen::Matrix4d& T2,
                     double angle_threshold_deg = 1,
                     double translation_threshold_m = 0.005,
                     bool output_error_cause = false);

/**
 * @brief Converts an array of float containing a pose measurement [R | t] (of
 * size 3 x 4) to a 4 x 4 Eigen Matrix
 * @param v vector of size 3 x 4 = 12
 * @return T
 */
Eigen::Matrix4d VectorToEigenTransform(const std::vector<float>& v);

/**
 * @brief outputs transform to some stream with as the following:
 *
 *  T
 *  RPY (0, 1, 2) in deg
 *  Quaternion(qw qx qy qz)
 *  translation (x y z)
 *
 * @param T transform to output
 * @param stream where to output to (optional)
 */
void OutputTransformInformation(const Eigen::Matrix4d& T,
                                std::ostream& stream = std::cout);

/** @} group utils */
} // namespace beam
