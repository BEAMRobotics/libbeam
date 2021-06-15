#include <beam_cv/geometry/RelativePoseEstimator.h>

#include <chrono>

#include <Eigen/Geometry>

#include <beam_cv/Utils.h>
#include <beam_cv/geometry/Triangulation.h>
#include <beam_utils/math.h>
#include <beam_utils/roots.h>

namespace beam_cv {

beam::opt<Eigen::Matrix3d> RelativePoseEstimator::EssentialMatrix8Point(
    const std::shared_ptr<beam_calibration::CameraModel>& cam1,
    const std::shared_ptr<beam_calibration::CameraModel>& cam2,
    const std::vector<Eigen::Vector2i, beam_cv::AlignVec2i>& p1_v,
    const std::vector<Eigen::Vector2i, beam_cv::AlignVec2i>& p2_v) {
  if (p2_v.size() < 8 || p1_v.size() < 8 || p1_v.size() != p2_v.size()) {
    BEAM_CRITICAL("Invalid number of input point matches.");
    return {};
  }
  int N = p2_v.size();
  // normalize input points via back projection
  std::vector<Eigen::Vector3d, beam_cv::AlignVec3d> X_r;
  std::vector<Eigen::Vector3d, beam_cv::AlignVec3d> X_c;
  for (int i = 0; i < N; i++) {
    Eigen::Vector3d xpr;
    Eigen::Vector3d xpc;
    if (cam1->BackProject(p1_v[i], xpr) && cam2->BackProject(p2_v[i], xpc)) {
      X_r.push_back(xpr);
      X_c.push_back(xpc);
    } else {
      BEAM_CRITICAL("Invalid pixel input. Unable to back project.");
      return {};
    }
  }
  // construct A matrix
  Eigen::MatrixXd A(N, 9);
  for (int i = 0; i < N; i++) {
    Eigen::VectorXd K = beam::KroneckerProduct(X_r[i], X_c[i]);
    A.row(i) = K;
  }
  // perform SVD on A matrix
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
  // singular vector of smallest singular value (9x1)
  Eigen::VectorXd x = svd.matrixV().col(8);
  // initial E estimate
  Eigen::MatrixXd Ea;
  beam::vec2mat(x, 3, 3, Ea);

  Eigen::JacobiSVD<Eigen::MatrixXd> svdEA(Ea, Eigen::ComputeFullV |
                                                  Eigen::ComputeFullU);
  Eigen::MatrixXd Va = svdEA.matrixV();
  Eigen::MatrixXd Ua = svdEA.matrixU();
  double s = (svdEA.singularValues()[0] + svdEA.singularValues()[1]) / 2;
  // determine algebraically best E (constrain to rank 2)
  Eigen::Vector3d D;
  D << s, s, 0;
  Eigen::Matrix3d E = Ua * D.asDiagonal() * Va.transpose();
  return E;
}

beam::opt<std::vector<Eigen::Matrix3d, beam_cv::AlignMat3d>>
    RelativePoseEstimator::EssentialMatrix7Point(
        const std::shared_ptr<beam_calibration::CameraModel>& cam1,
        const std::shared_ptr<beam_calibration::CameraModel>& cam2,
        const std::vector<Eigen::Vector2i, beam_cv::AlignVec2i>& p1_v,
        const std::vector<Eigen::Vector2i, beam_cv::AlignVec2i>& p2_v) {
  if (p2_v.size() < 7 || p1_v.size() < 7 || p1_v.size() != p2_v.size()) {
    BEAM_CRITICAL("Invalid number of input point matches.");
    return {};
  }
  int N = p2_v.size();
  // normalize input points via back projection
  std::vector<Eigen::Vector3d, beam_cv::AlignVec3d> X_r;
  std::vector<Eigen::Vector3d, beam_cv::AlignVec3d> X_c;
  for (int i = 0; i < N; i++) {
    Eigen::Vector3d xpr;
    Eigen::Vector3d xpc;
    if (cam1->BackProject(p1_v[i], xpr) && cam2->BackProject(p2_v[i], xpc)) {
      X_r.push_back(xpr);
      X_c.push_back(xpc);
    } else {
      BEAM_CRITICAL("Invalid pixel input. Unable to back project.");
      return {};
    }
  }
  // need vector of just normalized pixels (remove 3rd dim)
  // turn into a 2 x N matrix
  Eigen::MatrixXd matx1(2, N), matx2(2, N);
  // construct A matrix
  Eigen::MatrixXd A(N, 9);
  for (int i = 0; i < N; i++) {
    matx1.row(0)[i] = X_r[i][0];
    matx1.row(1)[i] = X_r[i][1];
    matx2.row(0)[i] = X_c[i][0];
    matx2.row(1)[i] = X_c[i][1];
    Eigen::VectorXd K = beam::KroneckerProduct(X_r[i], X_c[i]);
    A.row(i) = K;
  }
  // perform SVD on A matrix
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
  Eigen::VectorXd fvec1 = svd.matrixV().col(7);
  Eigen::VectorXd fvec2 = svd.matrixV().col(8);

  std::vector<Eigen::Matrix3d, beam_cv::AlignMat3d> Fmat(2);
  Fmat[0] << fvec1(0), fvec1(3), fvec1(6), fvec1(1), fvec1(4), fvec1(7),
      fvec1(2), fvec1(5), fvec1(8);
  Fmat[1] << fvec2(0), fvec2(3), fvec2(6), fvec2(1), fvec2(4), fvec2(7),
      fvec2(2), fvec2(5), fvec2(8);
  // find E that meets the singularity constraint: det(a * F1 + (1 - a) * F2) =
  // 0
  double D[2][2][2];
  for (int i1 = 0; i1 < 2; ++i1) {
    for (int i2 = 0; i2 < 2; ++i2) {
      for (int i3 = 0; i3 < 2; ++i3) {
        Eigen::Matrix3d Dtmp;
        Dtmp.col(0) = Fmat[i1].col(0);
        Dtmp.col(1) = Fmat[i2].col(1);
        Dtmp.col(2) = Fmat[i3].col(2);
        D[i1][i2][i3] = Dtmp.determinant();
      }
    }
  }
  // solving cubic equation and getting 1 or 3 solutions for E
  Eigen::VectorXd coefficients(4);
  coefficients(0) = -D[1][0][0] + D[0][1][1] + D[0][0][0] + D[1][1][0] +
                    D[1][0][1] - D[0][1][0] - D[0][0][1] - D[1][1][1];
  coefficients(1) = D[0][0][1] - 2 * D[0][1][1] - 2 * D[1][0][1] + D[1][0][0] -
                    2 * D[1][1][0] + D[0][1][0] + 3 * D[1][1][1];
  coefficients(2) = D[1][1][0] + D[0][1][1] + D[1][0][1] - 3 * D[1][1][1];
  coefficients(3) = D[1][1][1];
  // solve for roots of polynomial
  Eigen::VectorXd roots;
  beam::JenkinsTraubSolver solver(coefficients, &roots, NULL);
  solver.ExtractRoots();
  std::vector<Eigen::Matrix3d, beam_cv::AlignMat3d> E;
  // check sign consistency
  for (int i = 0; i < roots.size(); ++i) {
    Eigen::Matrix3d Ftmp = roots(i) * Fmat[0] + (1 - roots(i)) * Fmat[1];
    Eigen::JacobiSVD<Eigen::Matrix3d> fmatrix_svd(Ftmp.transpose(),
                                                  Eigen::ComputeFullV);
    Eigen::Vector3d e1 = fmatrix_svd.matrixV().col(2);
    // lines connecting of x1 and e1
    Eigen::Matrix<double, 3, Eigen::Dynamic> l1_ex =
        beam::SkewTransform(e1) * matx1.colwise().homogeneous();
    // lines determined by F and x2
    Eigen::Matrix<double, 3, Eigen::Dynamic> l1_Fx =
        Ftmp * matx2.colwise().homogeneous();

    Eigen::VectorXd s = (l1_Fx.array() * l1_ex.array()).colwise().sum();
    if ((s.array() > 0).all() || (s.array() < 0).all()) { E.push_back(Ftmp); }
  }
  return E;
}

beam::opt<Eigen::Matrix4d> RelativePoseEstimator::RANSACEstimator(
    const std::shared_ptr<beam_calibration::CameraModel>& cam1,
    const std::shared_ptr<beam_calibration::CameraModel>& cam2,
    const std::vector<Eigen::Vector2i, beam_cv::AlignVec2i>& p1_v,
    const std::vector<Eigen::Vector2i, beam_cv::AlignVec2i>& p2_v, EstimatorMethod method,
    int max_iterations, double inlier_threshold, int seed) {
  if (p2_v.size() != p1_v.size()) {
    BEAM_CRITICAL("Point match vectors are not of the same size.");
    return {};
  }
  // determine sample vector size
  uint32_t N = 0;
  if (method == EstimatorMethod::EIGHTPOINT) {
    N = 8;
  } else if (method == EstimatorMethod::SEVENPOINT) {
    N = 7;
  } else if (method == EstimatorMethod::FIVEPOINT) {
    N = 5;
  }
  // return nothing if not enough points
  if (p1_v.size() < N) {
    BEAM_CRITICAL("Not enough point correspondences, expecting at least {}", N);
    return {};
  }
  if (seed == -1) {
    srand(time(0));
  } else {
    srand(seed);
  }
  int current_inliers = 0;
  Eigen::Matrix4d current_pose;
  bool found_valid = false;
  for (int epoch = 0; epoch < max_iterations; epoch++) {
    std::vector<Eigen::Vector2i, beam_cv::AlignVec2i> pr_copy = p1_v;
    std::vector<Eigen::Vector2i, beam_cv::AlignVec2i> pc_copy = p2_v;
    std::vector<Eigen::Vector2i, beam_cv::AlignVec2i> sampled_pr;
    std::vector<Eigen::Vector2i, beam_cv::AlignVec2i> sampled_pc;
    // fill new point vectors with randomly sampled points from xs and xss
    int n = pr_copy.size();
    for (uint32_t i = 0; i < N; i++) {
      int idx = rand() % n;
      sampled_pr.push_back(pr_copy[idx]);
      sampled_pc.push_back(pc_copy[idx]);
      pr_copy.erase(pr_copy.begin() + idx);
      pc_copy.erase(pc_copy.begin() + idx);
      n--;
    }
    // perform estimation of the given method
    std::vector<Eigen::Matrix3d, beam_cv::AlignMat3d> Evec;
    if (method == EstimatorMethod::EIGHTPOINT) {
      beam::opt<Eigen::Matrix3d> E =
          RelativePoseEstimator::EssentialMatrix8Point(cam1, cam2, sampled_pr,
                                                       sampled_pc);
      if (E.has_value()) { Evec.push_back(E.value()); }
    } else if (method == EstimatorMethod::SEVENPOINT) {
      beam::opt<std::vector<Eigen::Matrix3d, beam_cv::AlignMat3d>> E =
          RelativePoseEstimator::EssentialMatrix7Point(cam1, cam2, sampled_pr,
                                                       sampled_pc);
      if (E.has_value()) { Evec = E.value(); }
    } else if (method == EstimatorMethod::FIVEPOINT) {
      BEAM_CRITICAL("Five point algorithm not yet implemented.");
      return {};
    }

    for (auto& E : Evec) {
      std::vector<Eigen::Matrix3d, beam_cv::AlignMat3d> R;
      std::vector<Eigen::Vector3d, beam_cv::AlignVec3d> t;
      RelativePoseEstimator::RtFromE(E, R, t);
      beam::opt<Eigen::Matrix4d> pose;
      int inliers = RelativePoseEstimator::RecoverPose(
          cam1, cam2, p1_v, p2_v, R, t, pose, inlier_threshold);

      if (pose.has_value()) {
        if (inliers > current_inliers) {
          found_valid = true;
          current_inliers = inliers;
          current_pose = pose.value();
        }
      }
    }
  }

  if (found_valid) {
    return current_pose;
  } else {
    return {};
  }
}

void RelativePoseEstimator::RtFromE(const Eigen::Matrix3d& E,
                                    std::vector<Eigen::Matrix3d, beam_cv::AlignMat3d>& R,
                                    std::vector<Eigen::Vector3d, beam_cv::AlignVec3d>& t) {
  Eigen::JacobiSVD<Eigen::Matrix3d> E_svd(E, Eigen::ComputeFullU |
                                                 Eigen::ComputeFullV);
  Eigen::Matrix3d U, V;
  U = E_svd.matrixU();
  V = E_svd.matrixV();
  Eigen::Matrix3d W;
  W << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  // two possible translation vectors
  t.resize(2);
  t[0] = U.block<3, 1>(0, 2);
  t[1] = -U.block<3, 1>(0, 2);
  // two possible rotation matrices
  R.resize(2);
  R[0] = U * W * V.transpose();
  R[1] = U * W.transpose() * V.transpose();
  // check determinant
  if (R[0].determinant() < 0) { R[0] = -R[0].eval(); }
  if (R[1].determinant() < 0) { R[1] = -R[1].eval(); }
}

int RelativePoseEstimator::RecoverPose(
    const std::shared_ptr<beam_calibration::CameraModel>& cam1,
    const std::shared_ptr<beam_calibration::CameraModel>& cam2,
    const std::vector<Eigen::Vector2i, beam_cv::AlignVec2i>& p1_v,
    const std::vector<Eigen::Vector2i, beam_cv::AlignVec2i>& p2_v,
    const std::vector<Eigen::Matrix3d, beam_cv::AlignMat3d>& R,
    const std::vector<Eigen::Vector3d, beam_cv::AlignVec3d>& t, beam::opt<Eigen::Matrix4d>& pose,
    double inlier_threshold) {
  Eigen::Matrix4d T_cam2_world;
  // iterate through each possibility
  int max_count = 0;
  int ret_inliers = 0;
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      // build relative pose possibility
      T_cam2_world.block<3, 3>(0, 0) = R[i];
      T_cam2_world.block<3, 1>(0, 3) = t[j].transpose();
      Eigen::Vector4d v{0, 0, 0, 1};
      T_cam2_world.row(3) = v;
      Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
      // triangulate correspondences
      std::vector<beam::opt<Eigen::Vector3d>> points =
          Triangulation::TriangulatePoints(cam1, cam2, I, T_cam2_world, p1_v,
                                           p2_v);
      int inliers = beam_cv::CheckInliers(cam1, cam2, p1_v, p2_v, points, I,
                                          T_cam2_world, inlier_threshold);
      int size = points.size();
      int count = 0;
      // check if each point is in front of both cameras
      for (int point_idx = 0; point_idx < size; point_idx++) {
        if (points[point_idx].has_value()) { count++; }
      }
      // kep track of pose that has the most points in front of both cameras
      if (count > max_count) {
        max_count = count;
        ret_inliers = inliers;
        pose = T_cam2_world;
      }
    }
  }
  if (max_count == 0) {
    pose = {};
    return -1;
  } else {
    return ret_inliers;
  }
}

} // namespace beam_cv