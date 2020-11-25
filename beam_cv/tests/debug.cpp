#include <beam_calibration/CameraModel.h>
#include <beam_cv/geometry/RelativePoseEstimator.h>
#include <beam_cv/geometry/Triangulation.h>
#include <beam_utils/math.hpp>
#include <opencv/cv.h>

#include <beam_calibration/Radtan.h>
#include <beam_cv/Raycast.h>
#include <beam_depth/DepthMap.h>
#include <beam_depth/Utils.h>
#include <beam_utils/utils.hpp>
#include <pcl/io/pcd_io.h>

using namespace cv;
using namespace std;

int main() {
  // file locations
  std::string intrinsics_loc = "/home/jake/K.json";
  // load other objects
  std::shared_ptr<beam_calibration::CameraModel> F1 =
      std::make_shared<beam_calibration::Radtan>(intrinsics_loc);

  Eigen::Vector2i p1(752, 1043);
  Eigen::Vector3d pp1 = 10 * F1->BackProject(p1).value();
  std::cout << p1[0] << "," << p1[1] << ":" << pp1[0] << "," << pp1[1] << ","
            << pp1[2] << std::endl;

  Eigen::Vector2i p2(628, 875);
  Eigen::Vector3d pp2 = 10 * F1->BackProject(p2).value();
  std::cout << p2[0] << "," << p2[2] << ":" << pp2[0] << "," << pp2[1] << ","
            << pp2[2] << std::endl;

  Eigen::Vector2i p3(1359, 1070);
  Eigen::Vector3d pp3 = 10 * F1->BackProject(p3).value();
  std::cout << p3[0] << "," << p3[1] << ":" << pp3[0] << "," << pp3[1] << ","
            << pp3[2] << std::endl;

  Eigen::Vector2i p4(452, 780);
  Eigen::Vector3d pp4 = 10 * F1->BackProject(p4).value();
  std::cout << p4[0] << "," << p4[1] << ":" << pp4[0] << "," << pp4[1] << ","
            << pp4[2] << std::endl;

  Eigen::Vector2i p5(708, 959);
  Eigen::Vector3d pp5 = 10 * F1->BackProject(p5).value();
  std::cout << p5[0] << "," << p5[1] << ":" << pp5[0] << "," << pp5[1] << ","
            << pp5[2] << std::endl;

  Eigen::Vector2i p6(655, 826);
  Eigen::Vector3d pp6 = 10 * F1->BackProject(p6).value();
  std::cout << p6[0] << "," << p6[1] << ":" << pp6[0] << "," << pp6[1] << ","
            << pp6[2] << std::endl;

  Eigen::Vector2i p7(859, 1034);
  Eigen::Vector3d pp7 = 10 * F1->BackProject(p7).value();
  std::cout << p7[0] << "," << p7[1] << ":" << pp7[0] << "," << pp7[1] << ","
            << pp7[2] << std::endl;

  Eigen::Vector2i p8(295, 338);
  Eigen::Vector3d pp8 = 10 * F1->BackProject(p1).value();
  std::cout << p8[0] << "," << p8[1] << ":" << pp8[0] << "," << pp8[1] << ","
            << pp8[2] << std::endl;

  Eigen::Vector2i p9(269, 180);
  Eigen::Vector3d pp9 = 10 * F1->BackProject(p9).value();
  std::cout << p9[0] << "," << p9[1] << ":" << pp9[0] << "," << pp9[1] << ","
            << pp9[2] << std::endl;
}
