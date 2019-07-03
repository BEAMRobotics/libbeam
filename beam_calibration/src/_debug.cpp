#include "beam_calibration/CameraModel.h"
#include "beam_calibration/LadybugCamera.h"
#include "beam_calibration/TfTree.h"
#include "beam_utils/math.hpp"
#include <boost/filesystem.hpp>
#include <typeinfo>

int main() {
  // radtan
  std::string radtan_location = __FILE__;
  radtan_location.erase(radtan_location.end() - 14, radtan_location.end());
  radtan_location += "tests/test_data/F2.json";
  std::shared_ptr<beam_calibration::CameraModel> radtan =
      beam_calibration::CameraModel::LoadJSON(radtan_location);

  beam::Vec3 point(-230, 4230, 304);
  std::cout << radtan->ProjectPoint(point) << std::endl;

  // load equidistant
  std::string equidistant_location = __FILE__;
  equidistant_location.erase(equidistant_location.end() - 14,
                             equidistant_location.end());
  equidistant_location += "tests/test_data/F2.json";
  std::shared_ptr<beam_calibration::CameraModel> equid =
      beam_calibration::CameraModel::LoadJSON(equidistant_location);
  beam::Vec2 p1(30, 100);
  beam::Vec2 p2 = equid->DistortPoint(p1);
  std::cout << "Original point: \n" << p1 << std::endl;
  std::cout << "Distorted point: \n" << p2 << std::endl;
  std::cout << "Undistorted point: \n"
            << equid->UndistortPoint(p2) << std::endl;

  beam::Vec3 p3(200, 200, 200);
  std::cout << "Projected: \n" << equid->ProjectPoint(p3) << std::endl;
  std::cout << "Projected undistorted: \n"
            << equid->ProjectUndistortedPoint(p3) << std::endl;

  // load conf file
  std::string ladybug_location = __FILE__;
  ladybug_location.erase(ladybug_location.end() - 14, ladybug_location.end());
  ladybug_location += "tests/test_data/ladybug.conf";
  beam_calibration::LadybugCamera ladybug(0, ladybug_location);
}
