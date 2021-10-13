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
void vec2mat(const std::vector<double>& x, int rows, int cols, Eigen::MatrixXd& y);

/** Reshapes a matrix to a vector*/
void mat2vec(const Eigen::MatrixXd& A, std::vector<double>& x);

/** Reshapes a vector `x` to matrix `y` of size `rows` and `cols` */
void vec2mat(const Eigen::VectorXd& x, int rows, int cols, Eigen::MatrixXd& y);

/** Reshapes a matrix to a vector*/
void mat2vec(const Eigen::MatrixXd& A, Eigen::VectorXd& x);

/** Convert euler angle to rotation matrix **/
int euler2rot(const Eigen::Vector3d& euler, int euler_seq, Eigen::Matrix3d& R);

/** Convert euler angle to quaternion **/
int euler2quat(const Eigen::Vector3d& euler, int euler_seq, Eigen::Quaterniond& q);

/** Convert quaternion to euler angles **/
int quat2euler(const Eigen::Quaterniond& q, int euler_seq, Eigen::Vector3d& euler);

/** Convert quaternion to rotation matrix **/
int quat2rot(const Eigen::Quaterniond& q, Eigen::Matrix3d& R);

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
Eigen::MatrixXd RoundMatrix(const Eigen::MatrixXd& M, int precision);

/**
 * @brief check if a matrix is a valid transformation matrix
 * @param T tranformation
 * @param summary reference to summary string
 * @param precision when checking R R^T == I, we round to this number of, same
 *with rounding det to make sure it is equal to 1 decimals
 **/
bool IsTransformationMatrix(const Eigen::Matrix4d& T,
                            std::string& summary = _tmp_string,
                            int precision = 3);

/**
 * @brief check if a matrix is a valid rotation matrix
 * @param R rotation matrix
 * @param summary reference to summary string
 * @param precision when checking R R^T == I, we round to this number of
 *decimals
 **/
bool IsRotationMatrix(const Eigen::Matrix3d& R,
                      std::string& summary = _tmp_string, int precision = 3);

/**
 * @brief Computes kronecker product in brute force manner
 * @param A n x m input matrix
 * @param B p x q input matrix
 * @return pm x qn matrix
 **/
Eigen::MatrixXd KroneckerProduct(const Eigen::MatrixXd& A,
                                 const Eigen::MatrixXd& B);

/**
 * @brief Convert from rotation matrix to its associated Lie Algebra
 * @param R rotation matrix
 * @return 3x1 vector representing R in Lie Algebra space
 **/
Eigen::Vector3d RToLieAlgebra(const Eigen::Matrix3d& R);

/**
 * @brief Convert from quaternion to its associated Lie Algebra
 * @param q quaternion
 * @return 3x1 vector representing R in Lie Algebra space
 **/
Eigen::Vector3d QToLieAlgebra(const Eigen::Quaterniond& q);

/**
 * @brief Convert from Lie Algebra to its associated rotation matrix
 * @param eps rotation in Lie Algebra space
 * @return rotation matrix
 **/
Eigen::Matrix3d LieAlgebraToR(const Eigen::Vector3d& eps);

/**
 * @brief Convert from Lie Algebra to its associated quaternion
 * @param eps rotation in Lie Algebra space
 * @return quaternion
 **/
Eigen::Quaterniond LieAlgebraToQ(const Eigen::Vector3d& eps);

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
Eigen::Matrix4d InterpolateTransform(const Eigen::Matrix4d& m1,
                                     const beam::TimePoint& t1,
                                     const Eigen::Matrix4d& m2,
                                     const beam::TimePoint& t2,
                                     const beam::TimePoint& t);

/**
 * @brief Perform inverse of skew symmetric transform
 * @param M 3x3 skew symmetric matrix
 * @return 3x1 vector
 **/
Eigen::Vector3d InvSkewTransform(const Eigen::Matrix3d& M);

/**
 * @brief Perform skew symmetric transform
 * @param V 3x1 vector
 * @return 3x3 skew symmetric matrix
 **/
Eigen::Matrix3d SkewTransform(const Eigen::Vector3d& V);

/**
 * @brief Inverts a 4x4 Transformation matrix by taking into account that
 *R^(-1)=R^T
 * @param T Transform matrix to be inverted
 * @return 4x4 inverted transformation matrix
 **/
Eigen::Matrix4d InvertTransform(const Eigen::MatrixXd& T);

/**
 * @brief Average a set of transforms by converting each to lie algebra space,
 *summing their quantities in this space, then dividing all by the number of
 *transforms and converting back to Lie Group
 * @param transforms vector of matrix4d transforms
 * @return averaged transform
 **/
Eigen::Matrix4d AverageTransforms(
    const std::vector<Eigen::Matrix4d, AlignMat4d>& transforms);

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
 * @brief Converts a vector of doubles containing a pose measurement [R | t] (of
 * size 3 x 4) to a 4 x 4 Eigen Matrix. The pose vector assumes the 3 x 4 matrix
 * is ordered left to right and top to bottom. Therefore, this will also work
 * for a standard 4 x 4 transformation matrix, because the final 4 values in the
 * matrix are unused.
 * @param v vector of size 3 x 4 = 12
 * @return T
 */
Eigen::Matrix4d VectorToEigenTransform(const std::vector<double>& v);

/**
 * @brief Converts a vector of floats containing a pose measurement [R | t] (of
 * size 3 x 4) to a 4 x 4 Eigen Matrix. The pose vector assumes the 3 x 4 matrix
 * is ordered left to right and top to bottom. Therefore, this will also work
 * for a standard 4 x 4 transformation matrix, because the final 4 values in the
 * matrix are unused.
 * @param v vector of size 3 x 4 = 12
 * @return T
 */
Eigen::Matrix4f VectorToEigenTransform(const std::vector<float>& v);

/**
 * @brief Converts an Eigen Matrix4d (transform) to a vector of 16 doubles. The
 * points are read from left to right, then down.
 * @param T transform of size 4 x 4
 * @return v vector of size 4 x 4 = 16
 */
std::vector<double> EigenTransformToVector(const Eigen::Matrix4d& T);

/**
 * @brief Converts an Eigen Matrix4d (transform) to a vector of 16 floats. The
 * points are read from left to right, then down.
 * @param T transform of size 4 x 4
 * @return v vector of size 4 x 4 = 16
 */
std::vector<float> EigenTransformToVector(const Eigen::Matrix4f& T);

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

/**
 * @brief Returns a string representation of a transformation matrix as:
 *  Roll: _, Pitch: _, Yaw: _, x: _, y: _, z: _
 * @param T transform
 */
std::string TransformationMatrixToString(const Eigen::Matrix4d& T);

/** @} group utils */
} // namespace beam
