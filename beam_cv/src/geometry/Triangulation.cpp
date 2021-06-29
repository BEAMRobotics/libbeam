#include <beam_cv/geometry/Triangulation.h>

#include <Eigen/Geometry>

namespace beam_cv {

beam::opt<Eigen::Vector3d> Triangulation::TriangulatePoint(
    const std::shared_ptr<beam_calibration::CameraModel>& cam1,
    const std::shared_ptr<beam_calibration::CameraModel>& cam2,
    const Eigen::Matrix4d& T_cam1_world, const Eigen::Matrix4d& T_cam2_world,
    const Eigen::Vector2i& p1, const Eigen::Vector2i& p2) {
  // we triangulate back projected points to be camera model invariant
  Eigen::Vector3d m1;
  Eigen::Vector3d m2;
  if (!cam1->BackProject(p1, m1) || !cam2->BackProject(p2, m2)) { return {}; }

  double mx1 = m1[0], my1 = m1[1], mz1 = m1[2];
  double mx2 = m2[0], my2 = m2[1], mz2 = m2[2];
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
  // check if result is in front of both cameras
  Eigen::Vector3d T_cam1_world_x = (T_cam1_world * x).hnormalized();
  Eigen::Vector3d T_cam2_world_x = (T_cam2_world * x).hnormalized();
  if (T_cam1_world_x[2] < 0 || T_cam2_world_x[2] < 0) { return {}; }
  Eigen::Vector3d xp = x.hnormalized();
  return xp;
}

beam::opt<Eigen::Vector3d> Triangulation::TriangulatePoint(
    const std::shared_ptr<beam_calibration::CameraModel>& cam,
    const std::vector<Eigen::Matrix4d, beam_cv::AlignMat4d>& T_cam_world,
    const std::vector<Eigen::Vector2i, beam_cv::AlignVec2i>& pixels,
    double reprojection_threshold) {
  if (pixels.size() != T_cam_world.size()) { return {}; }
  int rows = pixels.size() * 2;
  Eigen::MatrixXd A(rows, 4);
  for (uint32_t i = 0; i < pixels.size(); i++) {
    Eigen::Vector3d m;
    cam->BackProject(pixels[i], m);
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
  // check if result is in front of all cameras
  for (uint16_t i = 0; i < T_cam_world.size(); i++) {
    Eigen::Vector3d T_x = (T_cam_world[i] * x).hnormalized();
    // check if its behind the image plane
    if (T_x[2] < 0) { return {}; }
    // compute reprojection error
    if (reprojection_threshold != -1.0) {
      Eigen::Vector2d reproj_pixel;
      cam->ProjectPoint(T_x, reproj_pixel);
      Eigen::Vector2i reproj_pixeli = reproj_pixel.cast<int>();
      if (beam::distance(reproj_pixeli, pixels[i]) > reprojection_threshold) {
        return {};
      }
    }
  }

  Eigen::Vector3d xp = x.hnormalized();
  return xp;
}

std::vector<beam::opt<Eigen::Vector3d>> Triangulation::TriangulatePoints(
    const std::shared_ptr<beam_calibration::CameraModel>& cam1,
    const std::shared_ptr<beam_calibration::CameraModel>& cam2,
    const Eigen::Matrix4d& T_cam1_world, const Eigen::Matrix4d& T_cam2_world,
    const std::vector<Eigen::Vector2i, beam_cv::AlignVec2i>& p1_v,
    const std::vector<Eigen::Vector2i, beam_cv::AlignVec2i>& p2_v) {
  // loop through point vector and perform single point triangulation
  std::vector<beam::opt<Eigen::Vector3d>> result_pts3d;
  for (uint32_t i = 0; i < p1_v.size(); i++) {
    beam::opt<Eigen::Vector3d> pt3d = Triangulation::TriangulatePoint(
        cam1, cam2, T_cam1_world, T_cam2_world, p1_v[i], p2_v[i]);
    result_pts3d.push_back(pt3d);
  }
  return result_pts3d;
}
} // namespace beam_cv
