#include <beam_utils/math.h>

namespace beam {

int randi(int ub, int lb) {
  return rand() % (ub - lb + 1) + lb;
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

void vec2mat(const std::vector<double>& x, int rows, int cols,
             Eigen::MatrixXd& y) {
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

void mat2vec(const Eigen::MatrixXd& A, std::vector<double>& x) {
  for (int i = 0; i < A.cols(); i++) {
    for (int j = 0; j < A.rows(); j++) { x.push_back(A(i, j)); }
  }
}

void vec2mat(const Eigen::VectorXd& x, int rows, int cols, Eigen::MatrixXd& y) {
  int idx;
  // setup
  idx = 0;
  y.resize(rows, cols);

  // load matrix
  for (int i = 0; i < cols; i++) {
    for (int j = 0; j < rows; j++) {
      y(i, j) = x[idx];
      idx++;
    }
  }
}

void mat2vec(const Eigen::MatrixXd& A, Eigen::VectorXd& x) {
  std::vector<double> x_p;
  beam::mat2vec(A, x_p);
  x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(x_p.data(), x_p.size());
}

int euler2rot(const Eigen::Vector3d& euler, int euler_seq, Eigen::Matrix3d& R) {
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

int euler2quat(const Eigen::Vector3d& euler, int euler_seq,
               Eigen::Quaterniond& q) {
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

int quat2euler(const Eigen::Quaterniond& q, int euler_seq,
               Eigen::Vector3d& euler) {
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

int quat2rot(const Eigen::Quaterniond& q, Eigen::Matrix3d& R) {
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

void enu2nwu(const Eigen::Vector3d& enu, Eigen::Vector3d& nwu) {
  // ENU: (x - right, y - forward, z - up)
  // NWU: (x - forward, y - left, z - up)
  nwu(0) = enu(1);
  nwu(1) = -enu(0);
  nwu(2) = enu(2);
}

void ned2enu(const Eigen::Vector3d& ned, Eigen::Vector3d& enu) {
  // NED: (x - forward, y - right, z - down)
  // ENU: (x - right, y - forward, z - up)
  enu(0) = ned(1);
  enu(1) = ned(0);
  enu(2) = -ned(2);
}

void ned2nwu(const Eigen::Quaterniond& ned, Eigen::Quaterniond& nwu) {
  nwu.w() = ned.w();
  nwu.x() = ned.x();
  nwu.y() = -ned.y();
  nwu.z() = -ned.z();
}

void nwu2enu(const Eigen::Vector3d& nwu, Eigen::Vector3d& enu) {
  // NWU: (x - forward, y - left, z - up)
  // ENU: (x - right, y - forward, z - up)
  enu(0) = -nwu(1);
  enu(1) = nwu(0);
  enu(2) = nwu(2);
}

void nwu2ned(const Eigen::Quaterniond& nwu, Eigen::Quaterniond& ned) {
  ned.w() = nwu.w();
  ned.x() = nwu.x();
  ned.y() = -nwu.y();
  ned.z() = -nwu.z();
}

void nwu2edn(const Eigen::Vector3d& nwu, Eigen::Vector3d& edn) {
  // NWU: (x - forward, y - left, z - up)
  // EDN: (x - right, y - down, z - forward)
  edn(0) = -nwu(1);
  edn(1) = -nwu(2);
  edn(2) = nwu(0);
}

Eigen::Matrix3d RightJacobianOfSO3(const Eigen::Vector3d& w) {
  static const double root2_eps = sqrt(std::numeric_limits<double>::epsilon());
  static const double root4_eps = sqrt(root2_eps);
  static const double qdrt720 = sqrt(sqrt(720.0));
  static const double qdrt5040 = sqrt(sqrt(5040.0));
  static const double sqrt24 = sqrt(24.0);
  static const double sqrt120 = sqrt(120.0);

  double angle = w.norm();
  double cangle = cos(angle);
  double sangle = sin(angle);
  double angle2 = angle * angle;

  double cos_term;
  // compute (1-cos(x))/x^2, its taylor expansion around 0 is
  // 1/2-x^2/24+x^4/720+o(x^6)
  if (angle > root4_eps * qdrt720) {
    cos_term = (1 - cangle) / angle2;
  } else { // use taylor expansion to avoid singularity
    cos_term = 0.5;
    if (angle > root2_eps * sqrt24) { // we have to include x^2 term
      cos_term -= angle2 / 24.0;
    }
  }

  double sin_term;
  // compute (x-sin(x))/x^3, its taylor expansion around 0 is
  // 1/6-x^2/120+x^4/5040+o(x^6)
  if (angle > root4_eps * qdrt5040) {
    sin_term = (angle - sangle) / (angle * angle2);
  } else {
    sin_term = 1.0 / 6.0;
    if (angle > root2_eps * sqrt120) { // we have to include x^2 term
      sin_term -= angle2 / 120.0;
    }
  }

  Eigen::Matrix3d hat_w = beam::SkewTransform(w);
  return Eigen::Matrix3d::Identity() - cos_term * hat_w +
         sin_term * hat_w * hat_w;
}

Eigen::MatrixXd RoundMatrix(const Eigen::MatrixXd& M, int precision) {
  Eigen::MatrixXd Mround(M.rows(), M.cols());
  for (int i = 0; i < M.rows(); i++) {
    for (int j = 0; j < M.cols(); j++) {
      Mround(i, j) = std::round(M(i, j) * std::pow(10, precision)) /
                     std::pow(10, precision);
    }
  }
  return Mround;
}

bool IsTransformationMatrix(const Eigen::Matrix4d& T, std::string& summary,
                            int precision) {
  Eigen::Matrix3d R = T.block(0, 0, 3, 3);

  // check for infinity or nan
  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = 0; j < 4; j++) {
      if (std::isinf(T(i, j))) {
        summary = "Inf value found at coord. (" + std::to_string(i) + "" +
                  std::to_string(j) + ").";
        return false;
      }
      if (std::isnan(T(i, j))) {
        summary = "nan value found at coord. (" + std::to_string(i) + "" +
                  std::to_string(j) + ").";
        return false;
      }
    }
  }

  // check that bottom row is [0 0 0 1]
  if (T(3, 0) != 0 || T(3, 1) != 0 || T(3, 2) != 0 || T(3, 3) != 1) {
    summary = "Bottom row not equal to [0 0 0 1]";
    return false;
  }

  return IsRotationMatrix(R, summary, precision);
}

bool IsRotationMatrix(const Eigen::Matrix3d& R, std::string& summary,
                      int precision) {
  Eigen::Matrix3d shouldBeIdentity = RoundMatrix(R * R.transpose(), precision);
  if (!shouldBeIdentity.isIdentity()) {
    summary = "R R^T != Identity";
    return false;
  }

  double detR = R.determinant();
  double detRRound = std::round(detR * precision) / precision;
  if (detRRound != 1) {
    summary = "Det(R) != 1";
    return false;
  }

  return true;
}

Eigen::MatrixXd KroneckerProduct(const Eigen::MatrixXd& A,
                                 const Eigen::MatrixXd& B) {
  const int m = A.rows(), n = A.cols();
  const int p = B.rows(), q = B.cols();
  Eigen::MatrixXd C(p * m, q * n);
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) { C.block(i * p, j * q, p, q) = A(i, j) * B; }
  }
  return C;
}

Eigen::Matrix<double, 3, 2> S2TangentialBasis(const Eigen::Vector3d& x) {
  int d = 0;
  for (int i = 1; i < 3; ++i) {
    if (std::abs(x[i]) > std::abs(x[d])) d = i;
  }
  Eigen::Vector3d b1 = x.cross(Eigen::Vector3d::Unit((d + 1) % 3)).normalized();
  Eigen::Vector3d b2 = x.cross(b1).normalized();
  return (Eigen::Matrix<double, 3, 2>() << b1, b2).finished();
}

Eigen::Vector3d RToLieAlgebra(const Eigen::Matrix3d& R) {
  return InvSkewTransform(R.log());
}

Eigen::Vector3d QToLieAlgebra(const Eigen::Quaterniond& q) {
  Eigen::Matrix3d R = q.normalized().toRotationMatrix();
  return RToLieAlgebra(R);
}

Eigen::Matrix3d LieAlgebraToR(const Eigen::Vector3d& eps) {
  return SkewTransform(eps).exp();
}

Eigen::Quaterniond LieAlgebraToQ(const Eigen::Vector3d& eps) {
  Eigen::Quaterniond q(LieAlgebraToR(eps));
  return q.normalized();
}

Eigen::Matrix4d InterpolateTransform(const Eigen::Matrix4d& m1,
                                     const beam::TimePoint& t1,
                                     const Eigen::Matrix4d& m2,
                                     const beam::TimePoint& t2,
                                     const beam::TimePoint& t) {
  double w2 = 1.0 * (t - t1) / (t2 - t1);

  Eigen::Matrix4d T1 = m1;
  Eigen::Matrix4d T2 = m2;
  Eigen::Matrix4d T;

  Eigen::Matrix3d R1 = T1.block<3, 3>(0, 0);
  Eigen::Matrix3d R2 = T2.block<3, 3>(0, 0);
  Eigen::Matrix3d R = (R2 * R1.transpose()).pow(w2) * R1;

  Eigen::Vector4d tr1 = T1.rightCols<1>();
  Eigen::Vector4d tr2 = T2.rightCols<1>();
  Eigen::Vector4d tr = (1 - w2) * tr1 + w2 * tr2;

  T.setIdentity();
  T.block<3, 3>(0, 0) = R;
  T.rightCols<1>() = tr;

  return T;
}

Eigen::VectorXd InterpolateVector(const Eigen::VectorXd& v1, const double& t1,
                                  const Eigen::VectorXd& v2, const double& t2,
                                  const double& t) {
  if (v1.rows() != v2.rows()) {
    BEAM_ERROR("Input vectors must be of the same dimension!");
    throw std::runtime_error{"Invalid input vector dimensions."};
  }
  Eigen::VectorXd rise = v2 - v1;
  double run = t2 - t1;
  Eigen::VectorXd slope = rise / run;
  return slope * (t - t1);
}

Eigen::Vector3d InvSkewTransform(const Eigen::Matrix3d& M) {
  Eigen::Vector3d V;
  V(0) = M(2, 1);
  V(1) = M(0, 2);
  V(2) = M(1, 0);
  return V;
}

Eigen::Matrix3d SkewTransform(const Eigen::Vector3d& V) {
  Eigen::Matrix3d M;
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

Eigen::Matrix4d InvertTransform(const Eigen::MatrixXd& T) {
  Eigen::Matrix4d T_inv;
  T_inv.setIdentity();
  T_inv.block(0, 0, 3, 3) = T.block(0, 0, 3, 3).transpose();
  T_inv.block(0, 3, 3, 1) =
      -T.block(0, 0, 3, 3).transpose() * T.block(0, 3, 3, 1);
  return T_inv;
}

Eigen::Matrix4d RelativeTransform(const Eigen::Matrix4d& T_W_A,
                                  const Eigen::Matrix4d& T_W_B) {
  Eigen::Matrix4d W_T_A_B;
  Eigen::Matrix3d R_W_A = T_W_A.block<3, 3>(0, 0);
  Eigen::Matrix3d R_W_B = T_W_B.block<3, 3>(0, 0);
  Eigen::Matrix3d R_A_B = R_W_A.transpose() * R_W_B;
  Eigen::Vector3d t_A_B =
      T_W_A.block<3, 1>(0, 3).transpose() - T_W_B.block<3, 1>(0, 3).transpose();
  W_T_A_B.block<3, 3>(0, 0) = R_A_B;
  W_T_A_B.block<3, 1>(0, 3) = t_A_B.transpose();
  return W_T_A_B;
}

Eigen::Matrix4d AverageTransforms(
    const std::vector<Eigen::Matrix4d, AlignMat4d>& transforms) {
  if (transforms.size() == 1) { return transforms[0]; }

  std::vector<double> sum(6);
  for (const Eigen::Matrix4d T : transforms) {
    Eigen::Vector3d r = beam::RToLieAlgebra(T.block(0, 0, 3, 3));
    sum[0] += T(0, 3);
    sum[1] += T(1, 3);
    sum[2] += T(2, 3);
    sum[3] += r[0];
    sum[4] += r[1];
    sum[5] += r[2];
  }

  Eigen::Matrix4d T_AVG = Eigen::Matrix4d::Identity();
  Eigen::Vector3d r(sum[3] / transforms.size(), sum[4] / transforms.size(),
                    sum[5] / transforms.size());
  T_AVG.block(0, 0, 3, 3) = beam::LieAlgebraToR(r);
  T_AVG(0, 3) = sum[0] / transforms.size();
  T_AVG(1, 3) = sum[1] / transforms.size();
  T_AVG(2, 3) = sum[2] / transforms.size();

  return T_AVG;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d>
    FitPlane(const std::vector<Eigen::Vector3d>& c) {
  // copy coordinates to  matrix in Eigen format
  size_t num_atoms = c.size();
  Eigen::Matrix<Eigen::Vector3d::Scalar, Eigen::Dynamic, Eigen::Dynamic> coord(
      3, num_atoms);
  for (size_t i = 0; i < num_atoms; ++i) coord.col(i) = c[i];
  // calculate centroid
  Eigen::Vector3d centroid(coord.row(0).mean(), coord.row(1).mean(),
                           coord.row(2).mean());
  // subtract centroid
  coord.row(0).array() -= centroid(0);
  coord.row(1).array() -= centroid(1);
  coord.row(2).array() -= centroid(2);
  // we only need the left-singular matrix here
  //  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
  auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Vector3d plane_normal = svd.matrixU().rightCols<1>();
  return std::make_pair(centroid, plane_normal);
}

Eigen::Vector3d IntersectPoint(const Eigen::Vector3d& ray_vector,
                               const Eigen::Vector3d& ray_point,
                               const Eigen::Vector3d& plane_normal,
                               const Eigen::Vector3d& plane_point) {
  Eigen::Vector3d diff = ray_point - plane_point;
  double prod1 = diff.dot(plane_normal);
  double prod2 = ray_vector.dot(plane_normal);
  double prod3 = prod1 / prod2;
  return ray_point - ray_vector * prod3;
}

Eigen::Matrix4d PerturbTransformRadM(const Eigen::Matrix4d& T_in,
                                     const Eigen::VectorXd& perturbations) {
  Eigen::Vector3d r_perturb = perturbations.block(0, 0, 3, 1);
  Eigen::Vector3d t_perturb = perturbations.block(3, 0, 3, 1);
  Eigen::Matrix3d R_in = T_in.block(0, 0, 3, 3);
  Eigen::Matrix3d R_out = LieAlgebraToR(r_perturb) * R_in;
  Eigen::Matrix4d T_out;
  T_out.setIdentity();
  T_out.block(0, 3, 3, 1) = T_in.block(0, 3, 3, 1) + t_perturb;
  T_out.block(0, 0, 3, 3) = R_out;
  return T_out;
}

Eigen::Matrix4d PerturbTransformDegM(const Eigen::Matrix4d& T_in,
                                     const Eigen::VectorXd& perturbations) {
  Eigen::VectorXd perturbations_rad(perturbations);
  perturbations_rad[0] = beam::Deg2Rad(perturbations_rad[0]);
  perturbations_rad[1] = beam::Deg2Rad(perturbations_rad[1]);
  perturbations_rad[2] = beam::Deg2Rad(perturbations_rad[2]);
  return PerturbTransformRadM(T_in, perturbations_rad);
}

void RPYtoQuaternion(double roll, double pitch, double yaw,
                     Eigen::Quaterniond& q) {
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  q = rollAngle * pitchAngle * yawAngle;
  q.normalize();
}

void RPYtoQuaternion(const Eigen::Vector3d& rpy, Eigen::Quaterniond& q) {
  RPYtoQuaternion(rpy[0], rpy[1], rpy[2], q);
}

void RPYtoQuaternionDeg(double roll, double pitch, double yaw,
                        Eigen::Quaterniond& q) {
  RPYtoQuaternion(beam::Deg2Rad(roll), beam::Deg2Rad(pitch), beam::Deg2Rad(yaw),
                  q);
}

void RPYtoQuaternionDeg(const Eigen::Vector3d& rpy, Eigen::Quaterniond& q) {
  RPYtoQuaternion(beam::Deg2Rad(rpy[0]), beam::Deg2Rad(rpy[1]),
                  beam::Deg2Rad(rpy[2]), q);
}

void QuaterniontoRPY(const Eigen::Quaterniond& q, Eigen::Vector3d& rpy) {
  rpy = q.toRotationMatrix().eulerAngles(0, 1, 2); // roll pitch yaw order
}

void QuaterniontoRPY(const Eigen::Quaterniond& q, double& roll, double& pitch,
                     double& yaw) {
  Eigen::Vector3d rpy;
  QuaterniontoRPY(q, rpy);
  roll = rpy[0];
  pitch = rpy[1];
  yaw = rpy[2];
}

void QuaterniontoRPYDeg(const Eigen::Quaterniond& q, Eigen::Vector3d& rpy) {
  QuaterniontoRPY(q, rpy);
  rpy[0] = beam::Rad2Deg(rpy[0]);
  rpy[1] = beam::Rad2Deg(rpy[1]);
  rpy[2] = beam::Rad2Deg(rpy[2]);
}

void QuaterniontoRPYDeg(const Eigen::Quaterniond& q, double& roll,
                        double& pitch, double& yaw) {
  QuaterniontoRPY(q, roll, pitch, yaw);
  roll = beam::Rad2Deg(roll);
  pitch = beam::Rad2Deg(pitch);
  yaw = beam::Rad2Deg(yaw);
}

Eigen::Matrix4d
    QuaternionAndTranslationToTransformMatrix(const std::vector<double>& pose) {
  Eigen::Quaterniond quaternion{pose[0], pose[1], pose[2], pose[3]};
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block(0, 0, 3, 3) = quaternion.toRotationMatrix();
  T(0, 3) = pose[4];
  T(1, 3) = pose[5];
  T(2, 3) = pose[6];
  return T;
}

std::vector<double>
    TransformMatrixToQuaternionAndTranslation(const Eigen::Matrix4d& T) {
  Eigen::Matrix3d R = T.block(0, 0, 3, 3);
  Eigen::Quaterniond q = Eigen::Quaterniond(R);
  std::vector<double> pose{q.w(),   q.x(),   q.y(),  q.z(),
                           T(0, 3), T(1, 3), T(2, 3)};
  return pose;
}

void QuaternionAndTranslationToTransformMatrix(const Eigen::Quaterniond& q,
                                               const Eigen::Vector3d& p,
                                               Eigen::Matrix4d& T) {
  T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  T.block<3, 1>(0, 3) = p.transpose();
}

void TransformMatrixToQuaternionAndTranslation(const Eigen::Matrix4d& T,
                                               Eigen::Quaterniond& q,
                                               Eigen::Vector3d& p) {
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  Eigen::Quaterniond q_tmp(R);
  q = q_tmp;
  p = T.block<3, 1>(0, 3).transpose();
}

bool PassedMotionThreshold(const Eigen::Matrix4d& T1, const Eigen::Matrix4d& T2,
                           double angle_threshold_deg,
                           double translation_threshold_m,
                           bool is_threshold_minimum, bool must_pass_both,
                           bool output_error_cause) {
  Eigen::Matrix4d T_DIFF = beam::InvertTransform(T1) * T2;
  Eigen::Matrix3d R_DIFF = T_DIFF.block(0, 0, 3, 3);
  double angle =
      beam::Rad2Deg(std::abs(Eigen::AngleAxis<double>(R_DIFF).angle()));
  double translation = T_DIFF.block(0, 3, 3, 1).norm();

  if (is_threshold_minimum) {
    bool result1 = angle > angle_threshold_deg;
    bool result2 = translation > translation_threshold_m;
    if (!result1 && output_error_cause) {
      std::cout << "Angle: " << angle
                << " <= Threshold: " << angle_threshold_deg << "\n";
    }
    if (!result2 && output_error_cause) {
      std::cout << "Trans: " << translation
                << " <= Threshold: " << translation_threshold_m << "\n";
    }
    if (must_pass_both) {
      return result1 && result2;
    } else {
      return result1 || result2;
    }
  } else {
    bool result1 = angle < angle_threshold_deg;
    bool result2 = translation < translation_threshold_m;
    if (!result1 && output_error_cause) {
      std::cout << "Angle: " << angle
                << " >= Threshold: " << angle_threshold_deg << "\n";
    }
    if (!result2 && output_error_cause) {
      std::cout << "Trans: " << translation
                << " >= Threshold: " << translation_threshold_m << "\n";
    }
    if (must_pass_both) {
      return result1 && result2;
    } else {
      return result1 || result2;
    }
  }
}

bool ArePosesEqual(const Eigen::Matrix4d& T1, const Eigen::Matrix4d& T2,
                   double angle_threshold_deg, double translation_threshold_m,
                   bool output_error_cause) {
  return PassedMotionThreshold(T1, T2, angle_threshold_deg,
                               translation_threshold_m, false, true,
                               output_error_cause);
}

bool PassedMinMotion(const Eigen::Matrix4d& T1, const Eigen::Matrix4d& T2,
                     double angle_threshold_deg, double translation_threshold_m,
                     bool output_error_cause) {
  return PassedMotionThreshold(T1, T2, angle_threshold_deg,
                               translation_threshold_m, true, false,
                               output_error_cause);
}

Eigen::Matrix4f VectorToEigenTransform(const std::vector<float>& v) {
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0, 0) = v[0];
  T(0, 1) = v[1];
  T(0, 2) = v[2];
  T(0, 3) = v[3];
  T(1, 0) = v[4];
  T(1, 1) = v[5];
  T(1, 2) = v[6];
  T(1, 3) = v[7];
  T(2, 0) = v[8];
  T(2, 1) = v[9];
  T(2, 2) = v[10];
  T(2, 3) = v[11];
  return T;
}

Eigen::Matrix4d VectorToEigenTransform(const std::vector<double>& v) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 0) = v[0];
  T(0, 1) = v[1];
  T(0, 2) = v[2];
  T(0, 3) = v[3];
  T(1, 0) = v[4];
  T(1, 1) = v[5];
  T(1, 2) = v[6];
  T(1, 3) = v[7];
  T(2, 0) = v[8];
  T(2, 1) = v[9];
  T(2, 2) = v[10];
  T(2, 3) = v[11];
  return T;
}

std::vector<double> EigenTransformToVector(const Eigen::Matrix4d& T) {
  std::vector<double> v;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) { v.push_back(T(i, j)); }
  }
  return v;
}

std::vector<float> EigenTransformToVector(const Eigen::Matrix4f& T) {
  std::vector<float> v;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) { v.push_back(T(i, j)); }
  }
  return v;
}

void OutputTransformInformation(const Eigen::Matrix4d& T,
                                std::ostream& stream) {
  Eigen::Quaterniond q;
  Eigen::Vector3d t;
  TransformMatrixToQuaternionAndTranslation(T, q, t);
  Eigen::Matrix3d R = T.block(0, 0, 3, 3);
  Eigen::Vector3d rpy = R.eulerAngles(0, 1, 2);
  stream << "T: \n"
         << T << "\n"
         << "RPY (DEG): [" << Rad2Deg(rpy[0]) << ", " << Rad2Deg(rpy[1]) << ", "
         << Rad2Deg(rpy[3]) << "]\n"
         << "Quat (wxyz): [" << q.w() << ", " << q.x() << ", " << q.y() << ", "
         << q.z() << "]\n"
         << "Trans: [" << t[0] << ", " << t[1] << ", " << t[2] << "]\n";
}

std::string TransformationMatrixToString(const Eigen::Matrix4d& T) {
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  Eigen::Vector3d p = T.block<3, 1>(0, 3).transpose();
  Eigen::Vector3d ea = R.eulerAngles(2, 1, 0);
  std::string output;
  output += "Roll: " + std::to_string(ea[0]);
  output += ", Pitch: " + std::to_string(ea[1]);
  output += ", Yaw: " + std::to_string(ea[2]);
  output += ", x: " + std::to_string(p[0]);
  output += ", y: " + std::to_string(p[1]);
  output += ", z: " + std::to_string(p[2]);
  return output;
}

double Logit(double p) {
  return log(p / (1 - p));
}

double LogitInv(double l) {
  return exp(l) / (1 + exp(l));
}

double BayesianLogitUpdate(double pk, double l0, double p_prev) {
  double l_prev = Logit(p_prev);
  double l_updated = Logit(pk) + l_prev - l0;
  return LogitInv(l_updated);
}

} // namespace beam
