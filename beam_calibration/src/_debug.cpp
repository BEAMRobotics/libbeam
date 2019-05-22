#include "beam_calibration/CameraModel.h"
#include "beam_calibration/DistortionModel.h"
#include "beam_calibration/LadybugCamera.h"
#include "beam_calibration/PinholeCamera.h"
#include "beam_calibration/RadTanDistortion.h"
#include "beam_calibration/TfTree.h"
#include "beam_utils/math.hpp"
#include <boost/filesystem.hpp>
#include <typeinfo>

int main() {
  // load intrinsics
  std::string intrinsics_name = "F1.json";
  std::string intrinsics_location = __FILE__;
  intrinsics_location.erase(intrinsics_location.end() - 14,
                            intrinsics_location.end());
  intrinsics_location += "tests/test_data/";
  intrinsics_location += intrinsics_name;
  // load conf file
  std::string conf_name = "ladybug.conf";
  std::string conf_location = __FILE__;
  conf_location.erase(conf_location.end() - 14, conf_location.end());
  conf_location += "tests/test_data/";
  conf_location += conf_name;

  beam_calibration::LadybugCamera camera(0, conf_location);
}
