#include "beam_calibration/CameraModel.h"
#include "beam_calibration/LadybugCamera.h"
#include "beam_calibration/TfTree.h"
#include "beam_utils/math.hpp"
#include <boost/filesystem.hpp>
#include <typeinfo>

int main() {
  beam::Vec3 p0(123, 741, 7023);
  beam::Vec3 p1(34, 1234, 23);
  beam::Vec3 p2(1223, 10, 523);
  beam::Vec3 p3(12, 74, 70);
  // radtan
  std::string radtan_location = __FILE__;
  radtan_location.erase(radtan_location.end() - 14, radtan_location.end());
  radtan_location += "tests/test_data/F1.json";
  std::shared_ptr<beam_calibration::CameraModel> radtan =
      beam_calibration::CameraModel::LoadJSON(radtan_location);
  beam::Vec2 up(20, 30);
  // load equidistant
  std::string equidistant_location = __FILE__;
  equidistant_location.erase(equidistant_location.end() - 14,
                             equidistant_location.end());
  equidistant_location += "tests/test_data/F2.json";
  std::shared_ptr<beam_calibration::CameraModel> equid =
      beam_calibration::CameraModel::LoadJSON(equidistant_location);

  // load conf file
  std::string conf_location = __FILE__;
  conf_location.erase(conf_location.end() - 14, conf_location.end());
  conf_location += "tests/test_data/ladybug.conf";
  beam_calibration::LadybugCamera ladybug(0, conf_location);

  /**
   * To do:
   * - test making camera's manually
   * - test making children from the parent class
   *     - ie make cameramodel with its constructor then try to set it to
   * pinhole
   * - test setters and projections
   * - test undisort image
   * - test making distortion model's manually
   */
}
