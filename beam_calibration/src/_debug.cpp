#include "beam_calibration/CameraModel.h"
#include "beam_calibration/DistortionModel.h"
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

  auto camera = beam_calibration::CameraModel::LoadJSON(intrinsics_location);
}
