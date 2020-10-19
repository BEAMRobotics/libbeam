#include "beam_cv/geometry/PoseEstimator.h"
#include "beam_cv/geometry/Triangulation.h"
#include "beam_utils/math.hpp"

#include <Eigen/Geometry>

#include <chrono>
#include <cstdlib>

namespace beam_cv {

opt<Eigen::Matrix3d> PoseEstimator::EssentialMatrix8Point(
    std::shared_ptr<beam_calibration::CameraModel> camR,
    std::shared_ptr<beam_calibration::CameraModel> camC,
    std::vector<Eigen::Vector2i> xs, std::vector<Eigen::Vector2i> xss) {
  const int N = 8;
  if (xs.size() != N || xss.size() != N) {
    BEAM_CRITICAL("Invalid number of input point matches.");
    return {};
  }
  // normalize input points via back projection
  std::vector<Eigen::Vector3d> Xs;
  std::vector<Eigen::Vector3d> Xss;
  for (int i = 0; i < N; i++) {
    opt<Eigen::Vector3d> xpr = camR->BackProject(xs[i]);
    opt<Eigen::Vector3d> xpc = camC->BackProject(xss[i]);
    if (xpc.has_value() && xpr.has_value()) {
      Xs.push_back(xpr.value());
      Xss.push_back(xpc.value());
    } else {
      BEAM_CRITICAL("Invalid pixel input. Unable to back project.");
      return {};
    }
  }
  // construct A matrix
  Eigen::MatrixXd A(N, 9);
  for (int i = 0; i < N; i++) {
    Eigen::MatrixXd K = beam::KroneckerProduct(Xs[i], Xss[i]);
    Eigen::VectorXd ai;
    beam::mat2vec(K, ai);
    A.row(i) = ai;
  }
  // perform SVD on A matrix
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
  // singular vector of smallest singular value (9x1)
  Eigen::VectorXd x = svd.matrixV().col(8);
  // initial E estimate
  Eigen::MatrixXd Ea;
  beam::vec2mat(x, 3, 3, Ea);
  // SVD decomp of E
  Eigen::JacobiSVD<Eigen::MatrixXd> svdEA(Ea, Eigen::ComputeFullV |
                                                  Eigen::ComputeFullU);
  Eigen::MatrixXd Va = svdEA.matrixV();
  Eigen::MatrixXd Ua = svdEA.matrixU();
  // determine algebraically best E (constrain to rank 2)
  Eigen::Vector3d D;
  D << 1, 1, 0;
  Eigen::MatrixXd E = Ua * D.asDiagonal() * Va;
  return E;
}

opt<Eigen::Matrix3d> PoseEstimator::EssentialMatrix7Point(
    std::shared_ptr<beam_calibration::CameraModel> camR,
    std::shared_ptr<beam_calibration::CameraModel> camC,
    std::vector<Eigen::Vector2i> xs, std::vector<Eigen::Vector2i> xss) {
  // TODO
}

opt<Eigen::Matrix4d> PoseEstimator::RANSACEstimator(
    std::shared_ptr<beam_calibration::CameraModel> camR,
    std::shared_ptr<beam_calibration::CameraModel> camC,
    std::vector<Eigen::Vector2i> xs, std::vector<Eigen::Vector2i> xss,
    EstimatorMethod method, int max_iterations, double inlier_threshold) {
  if (xs.size() != xss.size()) {
    BEAM_CRITICAL("Point match vectors are not of the same size.");
    return {};
  }
  int current_inliers = 0;
  Eigen::Matrix4d current_pose;
  // seed random num generator with time
  srand(time(0));
  for (int epoch = 0; epoch < max_iterations; epoch++) {
    std::vector<Eigen::Vector2i> xs_copy = xs;
    std::vector<Eigen::Vector2i> xss_copy = xss;
    // determine sample vector size
    int N;
    if (method == EstimatorMethod::EIGHTPOINT) {
      N = 8;
    } else if (method == EstimatorMethod::SEVENPOINT) {
      N = 7;
    } else if (method == EstimatorMethod::FIVEPOINT) {
      N = 5;
    }
    // fill new point vectors with randomly sampled points from xs and xss
    std::vector<Eigen::Vector2i> sampled_xs;
    std::vector<Eigen::Vector2i> sampled_xss;
    int n = xs_copy.size();
    for (int i = 0; i < N; i++) {
      int idx = rand() % n;
      sampled_xs.push_back(xs_copy[idx]);
      xs_copy.erase(xs_copy.begin() + idx);
      sampled_xss.push_back(xss_copy[idx]);
      xss_copy.erase(xss_copy.begin() + idx);
      n--;
    }
    // perform pose estimation of the given method
    opt<Eigen::Matrix3d> E;
    if (method == EstimatorMethod::EIGHTPOINT) {
      E = PoseEstimator::EssentialMatrix8Point(camR, camC, sampled_xs,
                                               sampled_xss);
    } else if (method == EstimatorMethod::SEVENPOINT) {
    } else if (method == EstimatorMethod::FIVEPOINT) {
    }
    // recover pose from estimated essential matrix
    if (!E.has_value()) { continue; }
    std::vector<Eigen::Matrix3d> R;
    std::vector<Eigen::Vector3d> t;
    PoseEstimator::RtFromE(E.value(), R, t);
    Eigen::Matrix4d pose =
        PoseEstimator::RecoverPose(camR, camC, sampled_xs, sampled_xss, R, t);
    // check number of inliers and update current best estimate
    int inliers = PoseEstimator::CheckInliers(camR, camC, xs, xss, pose,
                                              inlier_threshold);
    if (inliers > current_inliers) {
      current_inliers = inliers;
      current_pose = pose;
    }
  }
}

void PoseEstimator::RtFromE(Eigen::Matrix3d E, std::vector<Eigen::Matrix3d>& R,
                            std::vector<Eigen::Vector3d>& t) {
  // decompose E into 4 possibilities
}

Eigen::Matrix4d PoseEstimator::RecoverPose(
    std::shared_ptr<beam_calibration::CameraModel> camR,
    std::shared_ptr<beam_calibration::CameraModel> camC,
    std::vector<Eigen::Vector2i> xs, std::vector<Eigen::Vector2i> xss,
    std::vector<Eigen::Matrix3d>& R, std::vector<Eigen::Vector3d>& t) {
  // triangulate points used to find E for each possible transform
  // return the transform where all points are in front of both cameras
}

int PoseEstimator::CheckInliers(
    std::shared_ptr<beam_calibration::CameraModel> camR,
    std::shared_ptr<beam_calibration::CameraModel> camC,
    std::vector<Eigen::Vector2i> pr_v, std::vector<Eigen::Vector2i> pc_v,
    Eigen::Matrix4d T, double inlier_threshold) {
  int inliers = 0;
  Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
  // triangulate correspondences
  std::vector<opt<Eigen::Vector3d>> points =
      Triangulation::TriangulatePoints(camR, camC, I, T, pr_v, pc_v);
  // reproject triangulated points and find their error
  for (size_t i = 0; i < points.size(); i++) {
    opt<Eigen::Vector2d> pr_rep = camR->ProjectPointPrecise(points[i].value());
    opt<Eigen::Vector2d> pc_rep = camC->ProjectPointPrecise(points[i].value());
    if (!pr_rep.has_value() || !pc_rep.has_value()) { continue; }
    Eigen::Vector2d pr_d{pr_v[i][0], pr_v[i][1]};
    Eigen::Vector2d pc_d{pc_v[i][0], pc_v[i][1]};
    double dist_c = beam::distance(pc_rep.value(), pc_d);
    double dist_r = beam::distance(pr_rep.value(), pr_d);
    if (dist_c < inlier_threshold && dist_r < inlier_threshold) { inliers++; }
  }
  return inliers;
}

} // namespace beam_cv