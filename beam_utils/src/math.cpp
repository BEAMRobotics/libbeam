#include "beam_utils/math.hpp"

namespace beam {

int randi(int ub, int lb) {
  return rand() % lb + ub;
}

double randf(double ub, double lb) {
  double f = (double)rand() / RAND_MAX;
  return lb + f * (ub - lb);
}

int fltcmp(double f1, double f2, double threshold) {
  if (fabs(f1 - f2) <= threshold) {
    return 0;
  } else if (f1 > f2) {
    return 1;
  } else {
    return -1;
  }
}

double median(std::vector<double> v) {
  double a, b;

  // sort values
  std::sort(v.begin(), v.end());

  // obtain median
  if (v.size() % 2 == 1) {
    // return middle value
    return v[v.size() / 2];

  } else {
    // grab middle two values and calc mean
    a = v[v.size() / 2];
    b = v[(v.size() / 2) - 1];
    return (a + b) / 2.0;
  }
}

int gcd(int a, int b) {
  if (b == 0) return a;
  return gcd(b, a % b);
}

cv::Mat GetCrossKernel(int size) {
  const cv::Mat kernel =
      cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(size, size));
  return kernel;
}

cv::Mat GetFullKernel(int size) {
  cv::Mat kernel = cv::Mat::ones(size, size, CV_8U);
  return kernel;
}

cv::Mat GetEllipseKernel(int size) {
  const cv::Mat kernel =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(size, size));
  return kernel;
}

void vec2mat(std::vector<double> x, int rows, int cols, MatX& y) {
  int idx;

  // setup
  idx = 0;
  y.resize(rows, cols);

  // load matrix
  for (int i = 0; i < cols; i++) {
    for (int j = 0; j < rows; j++) {
      y(j, i) = x[idx];
      idx++;
    }
  }
}

void mat2vec(MatX A, std::vector<double>& x) {
  for (int i = 0; i < A.cols(); i++) {
    for (int j = 0; j < A.rows(); j++) { x.push_back(A(j, i)); }
  }
}

int euler2rot(Vec3 euler, int euler_seq, Mat3& R) {
  double R11, R12, R13, R21, R22, R23, R31, R32, R33;
  double phi, theta, psi;

  phi = euler(0);
  theta = euler(1);
  psi = euler(2);

  if (euler_seq == 321) {
    // euler 3-2-1
    R11 = cos(theta) * cos(psi);
    R12 = sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi);
    R13 = cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi);

    R21 = cos(theta) * sin(psi);
    R22 = sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi);
    R23 = cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi);

    R31 = -sin(theta);
    R32 = sin(phi) * cos(theta);
    R33 = cos(phi) * cos(theta);

  } else if (euler_seq == 123) {
    // euler 1-2-3
    R11 = cos(theta) * cos(psi);
    R12 = cos(theta) * sin(psi);
    R13 = -sin(theta);

    R21 = sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi);
    R22 = sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi);
    R23 = sin(phi) * cos(theta);

    R31 = cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi);
    R32 = cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi);
    R33 = cos(phi) * cos(theta);

  } else {
    return -1;
  }

  R << R11, R12, R13, R21, R22, R23, R31, R32, R33;

  return 0;
}

int euler2quat(Vec3 euler, int euler_seq, Quaternion& q) {
  double alpha, beta, gamma;
  double c1, c2, c3, s1, s2, s3;
  double w, x, y, z;

  alpha = euler(0);
  beta = euler(1);
  gamma = euler(2);

  c1 = cos(alpha / 2.0);
  c2 = cos(beta / 2.0);
  c3 = cos(gamma / 2.0);
  s1 = sin(alpha / 2.0);
  s2 = sin(beta / 2.0);
  s3 = sin(gamma / 2.0);

  switch (euler_seq) {
    case 123:
      // euler 1-2-3 to quaternion
      w = c1 * c2 * c3 - s1 * s2 * s3;
      x = s1 * c2 * c3 + c1 * s2 * s3;
      y = c1 * s2 * c3 - s1 * c2 * s3;
      z = c1 * c2 * s3 + s1 * s2 * c3;
      break;

    case 321:
      // euler 3-2-1 to quaternion
      w = c1 * c2 * c3 + s1 * s2 * s3;
      x = s1 * c2 * c3 - c1 * s2 * s3;
      y = c1 * s2 * c3 + s1 * c2 * s3;
      z = c1 * c2 * s3 - s1 * s2 * c3;
      break;

    default:
      printf("Error! Invalid euler sequence [%d]\n", euler_seq);
      return -1;
      break;
  }

  q.w() = w;
  q.x() = x;
  q.y() = y;
  q.z() = z;

  return 0;
}

int quat2euler(Quaternion q, int euler_seq, Vec3& euler) {
  double qw, qx, qy, qz;
  double qw2, qx2, qy2, qz2;
  double phi, theta, psi;

  qw = q.w();
  qx = q.x();
  qy = q.y();
  qz = q.z();

  qw2 = pow(qw, 2);
  qx2 = pow(qx, 2);
  qy2 = pow(qy, 2);
  qz2 = pow(qz, 2);

  if (euler_seq == 123) {
    // euler 1-2-3
    phi = atan2(2 * (qz * qw - qx * qy), (qw2 + qx2 - qy2 - qz2));
    theta = asin(2 * (qx * qz + qy * qw));
    psi = atan2(2 * (qx * qw - qy * qz), (qw2 - qx2 - qy2 + qz2));

  } else if (euler_seq == 321) {
    // euler 3-2-1
    phi = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2));
    theta = asin(2 * (qy * qw - qx * qz));
    psi = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2));

  } else {
    return -1;
  }

  euler << phi, theta, psi;
  return 0;
}

int quat2rot(Quaternion q, Mat3& R) {
  double qw = q.w();
  double qx = q.x();
  double qy = q.y();
  double qz = q.z();

  // double qw2 = pow(qw, 2);
  double qx2 = pow(qx, 2);
  double qy2 = pow(qy, 2);
  double qz2 = pow(qz, 2);

  // inhomogeneous form
  double R11 = 1 - 2 * qy2 - 2 * qz2;
  double R12 = 2 * qx * qy + 2 * qz * qw;
  double R13 = 2 * qx * qz - 2 * qy * qw;

  double R21 = 2 * qx * qy - 2 * qz * qw;
  double R22 = 1 - 2 * qx2 - 2 * qz2;
  double R23 = 2 * qy * qz + 2 * qx * qw;

  double R31 = 2 * qx * qz + 2 * qy * qw;
  double R32 = 2 * qy * qz - 2 * qx * qw;
  double R33 = 1 - 2 * qx2 - 2 * qy2;

  // // homogeneous form
  // R11 = qx2 + qx2 - qy2 - qz2;
  // R12 = 2 * (qx * qy - qw * qz);
  // R13 = 2 * (qw * qy + qx * qz);
  //
  // R21 = 2 * (qx * qy + qw * qz);
  // R22 = qw2 - qx2 + qy2 - qz2;
  // R23 = 2 * (qy * qz - qw * qx);
  //
  // R31 = 2 * (qx * qz - qw * qy);
  // R32 = 2 * (qw * qx + qy * qz);
  // R33 = qw2 - qx2 - qy2 + qz2;

  R << R11, R12, R13, R21, R22, R23, R31, R32, R33;

  return 0;
}

void enu2nwu(const Vec3& enu, Vec3& nwu) {
  // ENU: (x - right, y - forward, z - up)
  // NWU: (x - forward, y - left, z - up)
  nwu(0) = enu(1);
  nwu(1) = -enu(0);
  nwu(2) = enu(2);
}

void ned2enu(const Vec3& ned, Vec3& enu) {
  // NED: (x - forward, y - right, z - down)
  // ENU: (x - right, y - forward, z - up)
  enu(0) = ned(1);
  enu(1) = ned(0);
  enu(2) = -ned(2);
}

void ned2nwu(const Quaternion& ned, Quaternion& nwu) {
  nwu.w() = ned.w();
  nwu.x() = ned.x();
  nwu.y() = -ned.y();
  nwu.z() = -ned.z();
}

void nwu2enu(const Vec3& nwu, Vec3& enu) {
  // NWU: (x - forward, y - left, z - up)
  // ENU: (x - right, y - forward, z - up)
  enu(0) = -nwu(1);
  enu(1) = nwu(0);
  enu(2) = nwu(2);
}

void nwu2ned(const Quaternion& nwu, Quaternion& ned) {
  ned.w() = nwu.w();
  ned.x() = nwu.x();
  ned.y() = -nwu.y();
  ned.z() = -nwu.z();
}

void nwu2edn(const Vec3& nwu, Vec3& edn) {
  // NWU: (x - forward, y - left, z - up)
  // EDN: (x - right, y - down, z - forward)
  edn(0) = -nwu(1);
  edn(1) = -nwu(2);
  edn(2) = nwu(0);
}

MatX RoundMatrix(const MatX& M, int precision) {
  MatX Mround(M.rows(), M.cols());
  for (int i = 0; i < M.rows(); i++) {
    for (int j = 0; j < M.cols(); j++) {
      Mround(i, j) = std::round(M(i, j) * std::pow(10, precision)) /
                     std::pow(10, precision);
    }
  }
  return Mround;
}

bool IsTransformationMatrix(Eigen::Matrix4d T) {
  Eigen::Matrix3d R = T.block(0, 0, 3, 3);
  bool homoFormValid, tValid;

  // check translation for infinity or nan
  if (std::isinf(T(0, 3)) || std::isinf(T(1, 3)) || std::isinf(T(2, 3)) ||
      std::isnan(T(0, 3)) || std::isnan(T(1, 3)) || std::isnan(T(2, 3))) {
    tValid = 0;
  } else {
    tValid = 1;
  }

  // check that bottom row is [0 0 0 1]
  if (T(3, 0) == 0 && T(3, 1) == 0 && T(3, 2) == 0 && T(3, 3) == 1) {
    homoFormValid = 1;
  } else {
    homoFormValid = 0;
  }

  if (homoFormValid && tValid && IsRotationMatrix(R)) {
    return 1;
  } else {
    return 0;
  }
}

bool IsRotationMatrix(Eigen::Matrix3d R) {
  int precision = 3;
  Eigen::Matrix3d shouldBeIdentity = RoundMatrix(R * R.transpose(), precision);
  double detR = R.determinant();
  double detRRound = std::round(detR * precision) / precision;
  if (shouldBeIdentity.isIdentity() && detRRound == 1) {
    return 1;
  } else {
    return 0;
  }
}

Eigen::Vector3d RToLieAlgebra(const Eigen::Matrix3d R) {
  return invSkewTransform(R.log());
}

Eigen::Matrix3d LieAlgebraToR(const Eigen::Vector3d eps) {
  return skewTransform(eps).exp();
}

beam::Mat4 InterpolateTransform(const beam::Mat4& m1, const beam::TimePoint& t1,
                                const beam::Mat4& m2, const beam::TimePoint& t2,
                                const beam::TimePoint& t) {
  double w2 = 1.0 * (t - t1) / (t2 - t1);

  beam::Mat4 T1 = m1;
  beam::Mat4 T2 = m2;
  beam::Mat4 T;

  beam::Mat3 R1 = T1.block<3, 3>(0, 0);
  beam::Mat3 R2 = T2.block<3, 3>(0, 0);
  beam::Mat3 R = (R2 * R1.transpose()).pow(w2) * R1;

  beam::Vec4 tr1 = T1.rightCols<1>();
  beam::Vec4 tr2 = T2.rightCols<1>();
  beam::Vec4 tr = (1 - w2) * tr1 + w2 * tr2;

  T.setIdentity();
  T.block<3, 3>(0, 0) = R;
  T.rightCols<1>() = tr;

  return T;
}

beam::Vec3 invSkewTransform(const beam::Mat3 M) {
  Eigen::Vector3d V;
  V(0) = M(2, 1);
  V(1) = M(0, 2);
  V(2) = M(1, 0);
  return V;
}

beam::Mat3 skewTransform(const beam::Vec3 V) {
  beam::Mat3 M;
  M(0, 0) = 0;
  M(0, 1) = -V(2, 0);
  M(0, 2) = V(1, 0);
  M(1, 0) = V(2, 0);
  M(1, 1) = 0;
  M(1, 2) = -V(0, 0);
  M(2, 0) = -V(1, 0);
  M(2, 1) = V(0, 0);
  M(2, 2) = 0;
  return M;
}

std::pair<beam::Vec3, beam::Vec3> FitPlane(const std::vector<beam::Vec3>& c) {
  // copy coordinates to  matrix in Eigen format
  size_t num_atoms = c.size();
  Eigen::Matrix<beam::Vec3::Scalar, Eigen::Dynamic, Eigen::Dynamic> coord(
      3, num_atoms);
  for (size_t i = 0; i < num_atoms; ++i) coord.col(i) = c[i];
  // calculate centroid
  beam::Vec3 centroid(coord.row(0).mean(), coord.row(1).mean(),
                      coord.row(2).mean());
  // subtract centroid
  coord.row(0).array() -= centroid(0);
  coord.row(1).array() -= centroid(1);
  coord.row(2).array() -= centroid(2);
  // we only need the left-singular matrix here
  //  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
  auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  beam::Vec3 plane_normal = svd.matrixU().rightCols<1>();
  return std::make_pair(centroid, plane_normal);
}

beam::Vec3 IntersectPoint(beam::Vec3 ray_vector, beam::Vec3 ray_point,
                          beam::Vec3 plane_normal, beam::Vec3 plane_point) {
  beam::Vec3 diff = ray_point - plane_point;
  double prod1 = diff.dot(plane_normal);
  double prod2 = ray_vector.dot(plane_normal);
  double prod3 = prod1 / prod2;
  return ray_point - ray_vector * prod3;
}

} // namespace beam
