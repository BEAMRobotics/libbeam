#include "beam_calibration/include/Refactor/KannalaBrandt.h"

namespace beam_calibration {

KannalaBrandt::KannalaBrandt(const std::string& file_path) {
  type_ = CameraType::KANNALBRANDT;
  LoadJSON(file_path);
  fx_ = &intrinsics_[0];
  fy_ = &intrinsics_[1];
  cx_ = &intrinsics_[2];
  cy_ = &intrinsics_[3];
  k1_ = &intrinsics_[4];
  k2_ = &intrinsics_[5];
  k3_ = &intrinsics_[6];
  k4_ = &intrinsics_[7];
}

void KannalaBrandt::ProjectPoint(const Eigen::Vector3d& point,
                                 std::optional<Eigen::Vector2i>& pixel) {
  return;
}

void KannalaBrandt::ProjectPoint(const Eigen::Vector3d& point,
                                 std::optional<Eigen::Vector2i>& pixel,
                                 std::optional<Eigen::MatrixXd>& J) {
  return;
}

void KannalaBrandt::BackProject(const Eigen::Vector2i& pixel,
                                std::optional<Eigen::Vector3d>& ray) {
  return;
}

void KannalaBrandt::ValidateInputs() {
  if (intrinsics_.size() != intrinsics_size_[type_]) {
    BEAM_CRITICAL("Invalid number of intrinsics read. read: {}, required: {}",
                  intrinsics.size(), intrinsics_size_[type_]);
    throw std::invalid_argument{"Invalid number of instrinsics read."};
  }
}

} // namespace beam_calibration
