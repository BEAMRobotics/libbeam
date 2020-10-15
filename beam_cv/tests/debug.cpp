#include "beam_calibration/CameraModel.h"
#include "beam_calibration/TfTree.h"
#include "beam_cv/geometry/Triangulation.h"
#include "beam_utils/math.hpp"

using namespace std;

int main() {
  std::string cam_loc = __FILE__;
  cam_loc.erase(cam_loc.end() - 15, cam_loc.end());
  cam_loc += "tests/test_data/F2.json";
  std::shared_ptr<beam_calibration::CameraModel> cam =
      beam_calibration::CameraModel::Create(cam_loc);

  Eigen::Matrix4d Pr = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d Pc;
  Pc << 0.981858, -0.0972401, 0.162788, -0.999901, 0.100072, 0.994937,
      -0.00926934, -0.00368404, -0.161062, 0.0253916, 0.986618, 0.0136145, 0, 0,
      0, 1;
  Eigen::Vector2i pr(404, 522);
  Eigen::Vector2i pc(399, 546);

  opt<Eigen::Vector3d> pt3d =
      beam_cv::Triangulation::TriangulatePoint(cam, cam, Pr, Pc, pr, pc);
  if (pt3d.has_value()) {
    std::cout << "\nTriangulated point: \n"
              << pt3d.value() << "\n"
              << std::endl;
    Eigen::Vector4d pt3d_h;
    pt3d_h << pt3d.value()[0], pt3d.value()[1], pt3d.value()[2], 1;

    Eigen::Vector4d img1_pt3d_h = Pr * pt3d_h;
    Eigen::Vector3d img1_pt3d = img1_pt3d_h.head(3) / img1_pt3d_h(3);
    opt<Eigen::Vector2i> img1_point_reprojected = cam->ProjectPoint(img1_pt3d);
    std::cout << "\nTransformed into image 1: \n"
              << img1_pt3d << "\n"
              << std::endl;
    if (img1_point_reprojected.has_value()) {
      std::cout << img1_point_reprojected.value() << std::endl;
    }

    Eigen::Vector4d img5_pt3d_h = Pc * pt3d_h;
    Eigen::Vector3d img5_pt3d = img5_pt3d_h.head(3) / img5_pt3d_h(3);
    std::cout << "\nTransformed into image 2: \n"
              << img5_pt3d << "\n"
              << std::endl;
    opt<Eigen::Vector2i> img5_point_reprojected = cam->ProjectPoint(img5_pt3d);
    if (img5_point_reprojected.has_value()) {
      std::cout << img5_point_reprojected.value() << std::endl;
    }
  }
}