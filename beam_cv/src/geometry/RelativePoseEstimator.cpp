#include "beam_cv/geometry/RelativePoseEstimator.h"
#include "beam_cv/geometry/Triangulation.h"
#include "beam_utils/math.hpp"

#include <Eigen/Geometry>

#include <chrono>
#include <cstdlib>

namespace beam_cv {

opt<Eigen::Matrix3d> RelativePoseEstimator::EssentialMatrix8Point(
    std::shared_ptr<beam_calibration::CameraModel> camR,
    std::shared_ptr<beam_calibration::CameraModel> camC,
    std::vector<Eigen::Vector2i> pr_v, std::vector<Eigen::Vector2i> pc_v) {
  if (pc_v.size() < 8 || pr_v.size() < 8 || pr_v.size() != pc_v.size()) {
    BEAM_CRITICAL("Invalid number of input point matches.");
    return {};
  }
  int N = pc_v.size();
  // normalize input points via back projection
  std::vector<Eigen::Vector3d> X_r;
  std::vector<Eigen::Vector3d> X_c;
  for (int i = 0; i < N; i++) {
    opt<Eigen::Vector3d> xpr = camR->BackProject(pr_v[i]);
    opt<Eigen::Vector3d> xpc = camC->BackProject(pc_v[i]);
    if (xpc.has_value() && xpr.has_value()) {
      X_r.push_back(xpr.value());
      X_c.push_back(xpc.value());
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
  return Ea;
}

opt<Eigen::Matrix4d> RelativePoseEstimator::RANSACEstimator(
    std::shared_ptr<beam_calibration::CameraModel> camR,
    std::shared_ptr<beam_calibration::CameraModel> camC,
    std::vector<Eigen::Vector2i> pr_v, std::vector<Eigen::Vector2i> pc_v,
    EstimatorMethod method, int max_iterations, double inlier_threshold) {
  if (pc_v.size() != pr_v.size()) {
    BEAM_CRITICAL("Point match vectors are not of the same size.");
    return {};
  }
  // determine sample vector size
  int N = 0;
  if (method == EstimatorMethod::EIGHTPOINT) {
    N = 8;
  } else if (method == EstimatorMethod::SEVENPOINT) {
    N = 7;
  } else if (method == EstimatorMethod::FIVEPOINT) {
    N = 5;
  }
  // retur nothing if not enough points
  if (pr_v.size() < N) {
    BEAM_CRITICAL("Not enough point correspondences, expecting at least {}", N);
    return {};
  }
  int current_inliers = 0;
  Eigen::Matrix4d current_pose;
  // seed random num generator with time
  srand(time(0));
  for (int epoch = 0; epoch < max_iterations; epoch++) {
    std::vector<Eigen::Vector2i> pr_copy = pr_v;
    std::vector<Eigen::Vector2i> pc_copy = pc_v;
    // fill new point vectors with randomly sampled points from xs and xss
    std::vector<Eigen::Vector2i> sampled_pr;
    std::vector<Eigen::Vector2i> sampled_pc;
    int n = pr_copy.size();
    for (int i = 0; i < N; i++) {
      int idx = rand() % n;
      sampled_pr.push_back(pr_copy[idx]);
      pr_copy.erase(pr_copy.begin() + idx);
      sampled_pc.push_back(pc_copy[idx]);
      pc_copy.erase(pc_copy.begin() + idx);
      n--;
    }
    // perform pose estimation of the given method
    opt<Eigen::Matrix3d> E;
    if (method == EstimatorMethod::EIGHTPOINT) {
      E = RelativePoseEstimator::EssentialMatrix8Point(camR, camC, sampled_pr,
                                                       sampled_pc);
    } else if (method == EstimatorMethod::SEVENPOINT) {
    } else if (method == EstimatorMethod::FIVEPOINT) {
    }
    // recover pose from estimated essential matrix
    if (!E.has_value()) { continue; }
    std::vector<Eigen::Matrix3d> R;
    std::vector<Eigen::Vector3d> t;
    RelativePoseEstimator::RtFromE(E.value(), R, t);
    Eigen::Matrix4d pose = RelativePoseEstimator::RecoverPose(
        camR, camC, sampled_pr, sampled_pc, R, t);
    // check number of inliers and update current best estimate
    int inliers = RelativePoseEstimator::CheckInliers(camR, camC, pr_v, pc_v,
                                                      pose, inlier_threshold);
    if (inliers > current_inliers) {
      current_inliers = inliers;
      current_pose = pose;
    }
  }
  return current_pose;
}

void RelativePoseEstimator::RtFromE(Eigen::Matrix3d E,
                                    std::vector<Eigen::Matrix3d>& R,
                                    std::vector<Eigen::Vector3d>& t) {
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

Eigen::Matrix4d RelativePoseEstimator::RecoverPose(
    std::shared_ptr<beam_calibration::CameraModel> camR,
    std::shared_ptr<beam_calibration::CameraModel> camC,
    std::vector<Eigen::Vector2i> pr_v, std::vector<Eigen::Vector2i> pc_v,
    std::vector<Eigen::Matrix3d>& R, std::vector<Eigen::Vector3d>& t) {
  Eigen::Matrix4d pose;
  // iterate through each possibility
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      pose.block<3, 3>(0, 0) = R[i];
      pose.block<3, 1>(0, 3) = t[i].transpose();
      Eigen::Vector4d v{0, 0, 0, 1};
      pose.row(3) = v;
      Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
      // triangulate correspondences
      std::vector<opt<Eigen::Vector3d>> points =
          Triangulation::TriangulatePoints(camR, camC, I, pose, pr_v, pc_v);
      int size = points.size();
      int count = 0;
      // check if each point is in front of both cameras
      for (int point_idx = 0; point_idx < size; point_idx++) {
        Eigen::Vector4d pt_h;
        pt_h << points[point_idx].value()[0], points[point_idx].value()[1],
            points[point_idx].value()[2], 1;
        pt_h = pose * pt_h;
        Eigen::Vector3d ptc = pt_h.head(3) / pt_h(3);
        if (points[point_idx].value()[2] > 0 && ptc[2] > 0) { count++; }
      }
      // if all points are in front of both, return the pose
      if (count == size) { return pose; }
    }
  }
}

int RelativePoseEstimator::CheckInliers(
    std::shared_ptr<beam_calibration::CameraModel> camR,
    std::shared_ptr<beam_calibration::CameraModel> camC,
    std::vector<Eigen::Vector2i> pr_v, std::vector<Eigen::Vector2i> pc_v,
    Eigen::Matrix4d T_camC_world, double inlier_threshold) {
  int inliers = 0;
  Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
  // triangulate correspondences
  std::vector<opt<Eigen::Vector3d>> points =
      Triangulation::TriangulatePoints(camR, camC, I, T_camC_world, pr_v, pc_v);
  // reproject triangulated points and find their error
  for (size_t i = 0; i < points.size(); i++) {
    // transform point into camC coordinates
    Eigen::Vector4d pt_h;
    pt_h << points[i].value()[0], points[i].value()[1], points[i].value()[2], 1;
    pt_h = T_camC_world * pt_h;
    Eigen::Vector3d ptc = pt_h.head(3) / pt_h(3);

    opt<Eigen::Vector2d> pr_rep = camR->ProjectPointPrecise(points[i].value());
    opt<Eigen::Vector2d> pc_rep = camC->ProjectPointPrecise(ptc);
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