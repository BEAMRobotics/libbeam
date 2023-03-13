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
