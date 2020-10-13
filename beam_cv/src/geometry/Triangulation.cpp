#include "beam_cv/geometry/Triangulation.h"

namespace beam_cv {

opt<Eigen::Vector3d> Triangulation::TriangulatePoint(
    std::shared_ptr<beam_calibration::CameraModel> camR,
    std::shared_ptr<beam_calibration::CameraModel> camC, Eigen::Matrix4d Pr,
    Eigen::Matrix4d Pc, Eigen::Vector2i pr, Eigen::Vector2i pc) {
  opt<Eigen::Vector3d> mr = camR->BackProject(pr);
  opt<Eigen::Vector3d> mc = camC->BackProject(pc);
  if (!mr.has_value() || !mc.has_value()) { return {}; }

  double mxr = mr.value()[0], myr = mr.value()[1], mzr = mr.value()[2];
  double mxc = mc.value()[0], myc = mc.value()[1], mzc = mc.value()[2];

  std::cout << Pr << "\n----------------" << std::endl;
  std::cout << Pc << "\n----------------" << std::endl;
  Eigen::Vector4d Pr3 = Pr.row(2), Pr1 = Pr.row(0), Pr2 = Pr.row(1);
  Eigen::Vector4d Pc3 = Pc.row(2), Pc1 = Pc.row(0), Pc2 = Pc.row(1);

  Eigen::Vector4d row1 = (mxr * Pr3) - (mzr * Pr1);
  Eigen::Vector4d row2 = (myr * Pr3) - (mzr * Pr2);
  Eigen::Vector4d row3 = (mxc * Pc3) - (mzc * Pc1);
  Eigen::Vector4d row4 = (myc * Pc3) - (mzc * Pc2);

  Eigen::Matrix4d A;
  A << row1, row2, row3, row4;
  std::cout << A << std::endl;
  Eigen::Vector4d x;
  Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);
  x = svd.matrixV().col(A.cols() - 1);

  Eigen::Vector3d xp = x.head(3) / x(3);
  return xp;
}

opt<std::vector<Eigen::Vector3d>> Triangulation::TriangulatePoints(
    std::shared_ptr<beam_calibration::CameraModel> camR,
    std::shared_ptr<beam_calibration::CameraModel> camC, Eigen::Affine3d Pr,
    Eigen::Affine3d Pc,
    std::vector<std::tuple<Eigen::Vector2i, Eigen::Vector2i>> points) {}
} // namespace beam_cv