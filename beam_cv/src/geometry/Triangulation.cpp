#include <beam_cv/geometry/Triangulation.h>
#include <beam_utils/utils.h>

#include <Eigen/Geometry>

#include <cassert>

namespace beam_cv {

Eigen::Vector3d Triangulation::TriangulatePoint(
    const std::shared_ptr<beam_calibration::CameraModel>& cam1,
    const std::shared_ptr<beam_calibration::CameraModel>& cam2,
    const Eigen::Matrix4d& T_world_cam1, const Eigen::Matrix4d& T_world_cam2,
    const Eigen::Vector2i& p1, const Eigen::Vector2i& p2) {
  // we triangulate back projected points to be camera model invariant
  Eigen::Vector3d m1;
  Eigen::Vector3d m2;

  assert(cam1->BackProject(p1, m1) && 
         "Pixel must be within the image when attempting to triangulate!");
  assert(cam2->BackProject(p2, m2) &&
         "Pixel must be within the image when attempting to triangulate!");

  m1.normalize();
  m2.normalize();
  double mx1 = m1[0], my1 = m1[1], mz1 = m1[2];
  double mx2 = m2[0], my2 = m2[1], mz2 = m2[2];
  /* building the linear system for triangulation from here:
  https://www.mdpi.com/1424-8220/19/20/4494/htm */
  Eigen::Vector4d Pr1 = T_world_cam1.row(0), Pr2 = T_world_cam1.row(1),
                  Pr3 = T_world_cam1.row(2);
  Eigen::Vector4d Pl1 = T_world_cam2.row(0), Pl2 = T_world_cam2.row(1),
                  Pl3 = T_world_cam2.row(2);
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
  Eigen::Vector3d xp = x.hnormalized();
  return xp;
}

Eigen::Vector3d Triangulation::TriangulatePoint(
    const std::shared_ptr<beam_calibration::CameraModel>& cam,
    const std::vector<Eigen::Matrix4d, beam::AlignMat4d>& T_world_cam,
    const std::vector<Eigen::Vector2i, beam::AlignVec2i>& pixels) {
  assert(pixels.size() == T_world_cam.size() &&
         "Number of poses and measurements must be the same!");
  int rows = pixels.size() * 2;
  Eigen::MatrixXd A(rows, 4);
  for (uint32_t i = 0; i < pixels.size(); i++) {
    Eigen::Vector3d m;
    assert(cam->BackProject(pixels[i], m) &&
           "Pixel must be within the image when attempting to triangulate!");
    m.normalize();
    double mx = m[0], my = m[1], mz = m[2];
    Eigen::Matrix4d T = T_world_cam[i];
    Eigen::Vector4d P1 = T.row(0), P2 = T.row(1), P3 = T.row(2);
    A.row(2 * i) = (mx * P3) - (mz * P1);
    A.row((2 * i) + 1) = (my * P3) - (mz * P2);
  }
  /* Solve the system by finding the right nullspace of A using the SVD
  decomposition*/
  Eigen::Vector4d x;
  Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);
  x = svd.matrixV().col(A.cols() - 1);
  Eigen::Vector3d xp = x.hnormalized();
  return xp;
}

std::vector<Eigen::Vector3d, beam::AlignVec3d> Triangulation::TriangulatePoints(
    const std::shared_ptr<beam_calibration::CameraModel>& cam1,
    const std::shared_ptr<beam_calibration::CameraModel>& cam2,
    const Eigen::Matrix4d& T_world_cam1, const Eigen::Matrix4d& T_world_cam2,
    const std::vector<Eigen::Vector2i, beam::AlignVec2i>& p1_v,
    const std::vector<Eigen::Vector2i, beam::AlignVec2i>& p2_v) {
  // loop through point vector and perform single point triangulation
  std::vector<Eigen::Vector3d, beam::AlignVec3d> result_pts3d;
  for (uint32_t i = 0; i < p1_v.size(); i++) {
    Eigen::Vector3d pt3d = Triangulation::TriangulatePoint(
        cam1, cam2, T_world_cam1, T_world_cam2, p1_v[i], p2_v[i]);
    result_pts3d.push_back(pt3d);
  }
  return result_pts3d;
}
} // namespace beam_cv
