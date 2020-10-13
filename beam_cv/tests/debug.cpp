#include "beam_calibration/CameraModel.h"
#include "beam_calibration/TfTree.h"
#include "beam_cv/geometry/Triangulation.h"

using namespace std;

int main(const int argc, const char** argv) {
  std::string cam_loc = __FILE__;
  cam_loc.erase(cam_loc.end() - 15, cam_loc.end());
  cam_loc += "tests/test_data/F1.json";
  std::shared_ptr<beam_calibration::CameraModel> cam =
      beam_calibration::CameraModel::Create(cam_loc);

  std::string ext_loc = "/home/jake/data/Market_Square/extrinsics.json";
  beam_calibration::TfTree tree;
  tree.LoadJSON(ext_loc);

  std::string F3_LINK = "F3_link";
  std::string BASE_LINK = "base_link";

  Eigen::Matrix4d T_baselink1_world;
  T_baselink1_world << 0.9999990550257754, -0.001335612560349797,
      -0.0003257094484840433, 1.123411160371379, 0.001335545933411012,
      0.9999990872088688, -0.0002046912064068111, 0.0247793442220646,
      0.0003259825393256189, 0.0002042560130495023, 0.9999999260074299,
      0.002445963847704456, 0, 0, 0, 1;

  Eigen::Matrix4d T_baselink5_world;
  T_baselink5_world << 0.9941888226891187, 0.1073476081758299,
      -0.008066960948941956, 5.99014791070122, -0.1074141544309757,
      0.9941795054768119, -0.008325281842671608, 0.232858208625578,
      0.007126308153719414, 0.009143407942879087, 0.9999328046540368,
      0.01562642015872166, 0, 0, 0, 1;

  Eigen::Matrix4d T_F3Link_baselink =
      tree.GetTransformEigen(F3_LINK, BASE_LINK).matrix();

  Eigen::Matrix4d T_img1_world = T_F3Link_baselink * T_baselink1_world;
  Eigen::Matrix4d T_img5_world = T_F3Link_baselink * T_baselink5_world;
  Eigen::Vector2i img1_point(758, 488);
  Eigen::Vector2i img5_point(845, 579);

  opt<Eigen::Vector3d> pt3d = beam_cv::Triangulation::TriangulatePoint(
      cam, cam, T_img1_world, T_img5_world, img1_point, img5_point);
  if (pt3d.has_value()) {
    std::cout << pt3d.value() << "\n----------------" << std::endl;
    Eigen::Vector4d pt3d_h;
    pt3d_h << pt3d.value()[0], pt3d.value()[1], pt3d.value()[2], 1;

    Eigen::Vector4d img1_pt3d_h = T_img1_world * pt3d_h;
    Eigen::Vector3d img1_pt3d = img1_pt3d_h.head(3) / img1_pt3d_h(3);
    opt<Eigen::Vector2i> img1_point_reprojected = cam->ProjectPoint(img1_pt3d);
    std::cout << img1_pt3d << "\n----------------" << std::endl;
    if (img1_point_reprojected.has_value()) {
      std::cout << img1_point_reprojected.value() << std::endl;
    }

    Eigen::Vector4d img5_pt3d_h = T_img5_world * pt3d_h;
    Eigen::Vector3d img5_pt3d = img5_pt3d_h.head(3) / img5_pt3d_h(3);
    std::cout << img5_pt3d << "\n----------------" << std::endl;
    opt<Eigen::Vector2i> img5_point_reprojected = cam->ProjectPoint(img5_pt3d);
    if (img5_point_reprojected.has_value()) {
      std::cout << img5_point_reprojected.value() << std::endl;
    }
  }
}