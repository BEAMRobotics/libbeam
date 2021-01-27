#include <beam_cv/geometry/Triangulation.h>

#include <Eigen/Geometry>

namespace beam_cv {

opt<Eigen::Vector3d> Triangulation::TriangulatePoint(
      const std::shared_ptr<beam_calibration::CameraModel>& cam1,
      const std::shared_ptr<beam_calibration::CameraModel>& cam2,
      const Eigen::Matrix4d& T_cam1_world, const Eigen::Matrix4d& T_cam2_world,
      const Eigen::Vector2i& p1, const Eigen::Vector2i& p2) {
  // we triangulate back projected points to be camera model invariant
  opt<Eigen::Vector3d> m1 = cam1->BackProject(p1);
  opt<Eigen::Vector3d> m2 = cam2->BackProject(p2);
  if (!m1.has_value() || !m2.has_value()) { return {}; }
  double mx1 = m1.value()[0], my1 = m1.value()[1], mz1 = m1.value()[2];
  double mx2 = m2.value()[0], my2 = m2.value()[1], mz2 = m2.value()[2];
  /* building the linear system for triangulation from here:
  https://www.mdpi.com/1424-8220/19/20/4494/htm */
  Eigen::Vector4d Pr1 = T_cam1_world.row(0), Pr2 = T_cam1_world.row(1),
                  Pr3 = T_cam1_world.row(2);
  Eigen::Vector4d Pl1 = T_cam2_world.row(0), Pl2 = T_cam2_world.row(1),
                  Pl3 = T_cam2_world.row(2);
  Eigen::Matrix4d A;
  A.row(0) = (mx1 * Pr3) - (mz1 * Pr1);
  A.row(1) = (my1 * Pr3) - (mz1 * Pr2);
  A.row(2) = (mx2 * Pl3) - (mz2 * Pl1);
  A.row(3) = (my2 * Pl3) - (mz2 * Pl2);
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
    const std::vector<std::shared_ptr<beam_calibration::CameraModel>>& cams,
    const std::vector<Eigen::Matrix4d>& T_cam_world,
    const std::vector<Eigen::Vector2i>& pixels) {
  if (cams.size() != T_cam_world.size() || cams.size() != pixels.size()) {
    return {};
  }
  int rows = cams.size() * 2;
  Eigen::MatrixXd A(rows, 4);
  for (uint32_t i = 0; i < cams.size(); i++) {
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
    const std::shared_ptr<beam_calibration::CameraModel>& cam1,
    const std::shared_ptr<beam_calibration::CameraModel>& cam2,
    const Eigen::Matrix4d& T_cam1_world, const Eigen::Matrix4d& T_cam2_world,
    const std::vector<Eigen::Vector2i>& p1_v,
    const std::vector<Eigen::Vector2i>& p2_v) {
  // loop through point vector and perform single point triangulation
  std::vector<opt<Eigen::Vector3d>> result_pts3d;
  for (uint32_t i = 0; i < p1_v.size(); i++) {
    opt<Eigen::Vector3d> pt3d = Triangulation::TriangulatePoint(
        cam1, cam2, T_cam1_world, T_cam2_world, p1_v[i], p2_v[i]);
    result_pts3d.push_back(pt3d);
  }
  return result_pts3d;
}
} // namespace beam_cv