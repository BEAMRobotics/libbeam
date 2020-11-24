#include <beam_cv/geometry/Triangulation.h>

#include <Eigen/Geometry>

namespace beam_cv {

opt<Eigen::Vector3d> Triangulation::TriangulatePoint(
    std::shared_ptr<beam_calibration::CameraModel> camR,
    std::shared_ptr<beam_calibration::CameraModel> camC,
    Eigen::Matrix4d T_camR_world, Eigen::Matrix4d T_camC_world,
    Eigen::Vector2i pr, Eigen::Vector2i pc) {
  // we triangulate back projected points to be camera model invariant
  opt<Eigen::Vector3d> mr = camR->BackProject(pr);
  opt<Eigen::Vector3d> mc = camC->BackProject(pc);
  if (!mr.has_value() || !mc.has_value()) { return {}; }
  double mxr = mr.value()[0], myr = mr.value()[1], mzr = mr.value()[2];
  double mxc = mc.value()[0], myc = mc.value()[1], mzc = mc.value()[2];
  /* building the linear system for triangulation from here:
  https://www.mdpi.com/1424-8220/19/20/4494/htm */
  Eigen::Vector4d Pr1 = T_camR_world.row(0), Pr2 = T_camR_world.row(1),
                  Pr3 = T_camR_world.row(2);
  Eigen::Vector4d Pc1 = T_camC_world.row(0), Pc2 = T_camC_world.row(1),
                  Pc3 = T_camC_world.row(2);
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

opt<Eigen::Vector3d> Triangulation::TriangulatePoint(
    std::vector<std::shared_ptr<beam_calibration::CameraModel>> cams,
    std::vector<Eigen::Matrix4d> T_cam_world,
    std::vector<Eigen::Vector2i> pixels) {
  if (cams.size() != T_cam_world.size() || cams.size() != pixels.size()) {
    return {};
  }
  int rows = cams.size() * 2;
  Eigen::MatrixXd A(rows, 4);
  for (int i = 0; i < cams.size(); i++) {
    Eigen::Vector3d m = cams[i]->BackProject(pixels[i]).value();
    double mx = m[0], my = m[1], mz = m[2];
    Eigen::Matrix4d T = T_cam_world[i];
    Eigen::Vector4d P1 = T.row(0), P2 = T.row(1), P3 = T.row(2);
    A.row(2 * i) = (mx * P3) - (mz * P1);
    A.row((2 * i) + 1) = (my * P3) - (mz * P2);
  }
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
    std::shared_ptr<beam_calibration::CameraModel> camC,
    Eigen::Matrix4d T_camR_world, Eigen::Matrix4d T_camC_world,
    std::vector<Eigen::Vector2i> pr_v, std::vector<Eigen::Vector2i> pc_v) {
  // loop through point vector and perform single point triangulation
  std::vector<opt<Eigen::Vector3d>> result_pts3d;
  for (uint32_t i = 0; i < pr_v.size(); i++) {
    opt<Eigen::Vector3d> pt3d = Triangulation::TriangulatePoint(
        camR, camC, T_camR_world, T_camC_world, pr_v[i], pc_v[i]);
    result_pts3d.push_back(pt3d);
  }
  return result_pts3d;
}
} // namespace beam_cv