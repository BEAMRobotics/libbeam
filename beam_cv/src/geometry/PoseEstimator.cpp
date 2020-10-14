#include "beam_cv/geometry/PoseEstimator"

namespace beam_cv {

Eigen::Matrix3d PoseEstimator::EssentialMatrix8Point(
    std::shared_ptr<beam_calibration::CameraModel> camR,
    std::shared_ptr<beam_calibration::CameraModel> camC, Eigen::MatrixXd xs,
    Eigen::MatrixXd xss) {
  // back project points for "normalization"
  // build A matrix using kronecker product
  // compute svd of A
  // initial E estimate
  // SVD decomp of E
  // determine algebraically best E (constrain to rank 2)
}

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