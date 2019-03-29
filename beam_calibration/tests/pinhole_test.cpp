#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include "beam/utils/math.hpp"
#include "beam/calibration/Pinhole.h"

TEST_CASE("Defect type is returned", "[GetK]") {
  double fx = 10, fy = 2, cx = 3, cy = 4;
  beam::Mat3 K;
  K << fx, 0, cx,
       0, fy, cy,
       0, 0, 1;
  beam_calibration::Pinhole calib(fx, fy, cx, cy);

  REQUIRE(calib.GetK() == K);
}
