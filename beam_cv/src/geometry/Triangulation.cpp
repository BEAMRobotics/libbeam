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
  /* building the linear system for triangulation from here:
  https://www.mdpi.com/1424-8220/19/20/4494/htm */
  Eigen::Vector4d Pr1 = Pr.row(0), Pr2 = Pr.row(1), Pr3 = Pr.row(2);
  Eigen::Vector4d Pc1 = Pc.row(0), Pc2 = Pc.row(1), Pc3 = Pc.row(2);
  Eigen::Matrix4d A;
  A.row(0) = (mxr * Pr3) - (mzr * Pr1);
  A.row(1) = (myr * Pr3) - (mzr * Pr2);
  A.row(2) = (mxc * Pc3) - (mzc * Pc1);
  A.row(3) = (myc * Pc3) - (mzc * Pc2);
  /* Solve the system by finding the right nullspace of A using the SVD
  decomposition*/
  Eigen::Vector4d x;
  Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);
  x = svd.matrixV().col(A.cols() - 1);
  // normalize result to be in euclidean coordinates
  Eigen::Vector3d xp = x.head(3) / x(3);
  return xp;
}

std::vector<opt<Eigen::Vector3d>> Triangulation::TriangulatePoints(
    std::shared_ptr<beam_calibration::CameraModel> camR,
    std::shared_ptr<beam_calibration::CameraModel> camC, Eigen::Matrix4d Pr,
    Eigen::Matrix4d Pc,
    std::vector<std::tuple<Eigen::Vector2i, Eigen::Vector2i>> points) {
  std::vector<opt<Eigen::Vector3d>> result_pts3d;
  for (int i = 0; i < points.size(); i++) {
    std::tuple pair = points[i];
    Eigen::Vector2i pr = std::get<0>(pair);
    Eigen::Vector2i pc = std::get<1>(pair);
    opt<Eigen::Vector3d> pt3d =
        Triangulation::TriangulatePoint(camR, camC, Pr, Pc, pr, pc);
    result_pts3d.push_back(pt3d);
  }
  return result_pts3d;
}
} // namespace beam_cv