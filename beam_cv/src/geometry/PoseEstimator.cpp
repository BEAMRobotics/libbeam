#include "beam_cv/geometry/PoseEstimator.h"
#include "beam_utils/math.hpp"

#include <Eigen/Geometry>

namespace beam_cv {

opt<Eigen::Matrix3d> PoseEstimator::EssentialMatrix8Point(
    std::shared_ptr<beam_calibration::CameraModel> camR,
    std::shared_ptr<beam_calibration::CameraModel> camC, Eigen::MatrixXi xs,
    Eigen::MatrixXi xss) {
  if (xs.rows() != 8 || xs.cols() != 2 || xss.rows() != 8 || xss.cols() != 2) {
    BEAM_CRITICAL("Invalid input array dimensions");
    return {};
  }
  // normalize input points via back projection
  Eigen::MatrixXd Xs(8, 3);
  Eigen::MatrixXd Xss(8, 3);
  for (int i = 0; i < xs.rows(); i++) {
    Eigen::Vector2i pr = xs.row(i);
    Eigen::Vector2i pc = xss.row(i);
    opt<Eigen::Vector3d> xpr = camR->BackProject(pr);
    opt<Eigen::Vector3d> xpc = camC->BackProject(pc);
    if (xpc.has_value() && xpr.has_value()) {
      Xs.row(i) = xpr.value();
      Xss.row(i) = xpc.value();
    } else {
      BEAM_CRITICAL("Invalid pixel input. Unable to back project.");
      return {};
    }
  }
  // construct A matrix
  Eigen::MatrixXd A(Xs.rows(), 9);
  for (int i = 0; i < Xs.rows(); i++) {
    Eigen::MatrixXd K = beam::KroneckerProduct(Xs.row(i), Xss.row(i));
    Eigen::VectorXd ai;
    beam::mat2vec(K, ai);
    A.row(i) = ai;
  }
  // perform SVD on A matrix
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
  // singular vector of smallest singular value (9x1)
  Eigen::VectorXd x = svd.matrixV().col(A.cols() - 1);
  // initial E estimate
  Eigen::MatrixXd Ea;
  beam::vec2mat(x, 3, 3, Ea);
  // SVD decomp of E
  Eigen::JacobiSVD<Eigen::MatrixXd> svdEA(Ea, Eigen::ComputeFullV |
                                                  Eigen::ComputeFullU);
  Eigen::MatrixXd Va = svdEA.matrixV();
  Eigen::MatrixXd Ua = svdEA.matrixU();
  // determine algebraically best E (constrain to rank 2)
  Eigen::Vector3d diag;
  diag << 1, 1, 0;
  Eigen::MatrixXd E = Ua * diag.asDiagonal() * Va;
  return E;
}

opt<Eigen::Matrix3d> PoseEstimator::RANSACEstimator(
    std::shared_ptr<beam_calibration::CameraModel> camR,
    std::shared_ptr<beam_calibration::CameraModel> camC, Eigen::MatrixXi xs,
    Eigen::MatrixXi xss, EstimatorMethod method) {}

void PoseEstimator::RtFromE(Eigen::Matrix3d E, std::vector<Eigen::Matrix3d>& R,
                            std::vector<Eigen::Vector3d>& t) {
  // decompose E into 4 possibilities
}

Eigen::Matrix4d PoseEstimator::RecoverPose(
    std::shared_ptr<beam_calibration::CameraModel> camR,
    std::shared_ptr<beam_calibration::CameraModel> camC, Eigen::MatrixXd xs,
    Eigen::MatrixXd xss, std::vector<Eigen::Matrix3d>& R,
    std::vector<Eigen::Vector3d>& t) {
  // triangulate points used to find E for each possible transform
  // return the transform where all points are in front of both cameras
}

} // namespace beam_cv