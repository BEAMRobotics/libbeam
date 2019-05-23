#include "beam_calibration/CameraModel.h"
#include "beam_calibration/LadybugCamera.h"
#include "beam_calibration/TfTree.h"
#include "beam_utils/math.hpp"
#include <boost/filesystem.hpp>
#include <typeinfo>

int main() {
  // radtan
  std::string radtan_name = "F1.json";
  std::string radtan_location = __FILE__;
  radtan_location.erase(radtan_location.end() - 14, radtan_location.end());
  radtan_location += "tests/test_data/";
  radtan_location += radtan_name;

  std::shared_ptr<beam_calibration::CameraModel> camera =
      beam_calibration::CameraModel::LoadJSON(radtan_location);

  beam::Vec3 point(20, 20, 20);
  std::cout << camera->ProjectPoint(point) << std::endl;

  /*
  // load equidistant
  std::string equidistant_name = "F2.json";
  std::string equidistant_location = __FILE__;
  equidistant_location.erase(equidistant_location.end() - 14,
                            equidistant_location.end());
  equidistant_location += "tests/test_data/";
  equidistant_location += equidistant_name;

  std::shared_ptr<beam_calibration::CameraModel> camera2 =
      beam_calibration::CameraModel::LoadJSON(equidistant_location);
  beam::Vec3 pointx(20, 20, 20);
  std::cout << camera2->ProjectPoint(pointx) << std::endl;
  */

  /*
  // load conf file
  std::string conf_name = "ladybug.conf";
  std::string conf_location = __FILE__;
  conf_location.erase(conf_location.end() - 14, conf_location.end());
  conf_location += "tests/test_data/";
  conf_location += conf_name;

  beam_calibration::LadybugCamera ladybug(0, conf_location);

  beam::Vec3 point0{1034.3, 76.1, 4987.1};
  beam::Vec3 point1{600.34, 203.4253, 500.324};
  beam::Vec3 point2{1234.3, 49.1, 5303.1};
  beam::Vec3 point3{781.34, 183.4253, 398.324};
  std::cout << ladybug.ProjectPoint(point0) << std::endl;
  std::cout << ladybug.ProjectPoint(point1) << std::endl;
  std::cout << ladybug.ProjectPoint(point2) << std::endl;
  std::cout << ladybug.ProjectPoint(point3) << std::endl;
  */
}
