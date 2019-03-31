#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include "beam/utils/math.hpp"
#include "beam/calibration/Pinhole.h"

TEST_CASE("Test constructors and return funcitons") {
  double fx = 1, fy = 2, cx = 3, cy = 4;
  beam::Mat3 K;
  K << fx, 0, cx,
       0, fy, cy,
       0, 0, 1;
  beam_calibration::Pinhole calib(fx, fy, cx, cy);
  beam_calibration::Pinhole calib2(K);

  REQUIRE(calib.GetType() == beam_calibration::IntrinsicsType::PINHOLE);
  REQUIRE(calib.IsFull() == true);
  REQUIRE(calib.GetK() == K);
  REQUIRE(calib.GetFx() == fx);
  REQUIRE(calib.GetFy() == fy);
  REQUIRE(calib.GetCx() == cx);
  REQUIRE(calib.GetCy() == cy);

  REQUIRE(calib2.GetK() == K);
  REQUIRE(calib2.IsFull() == true);
  REQUIRE(calib2.GetFx() == fx);
  REQUIRE(calib2.GetFy() == fy);
  REQUIRE(calib2.GetCx() == cx);
  REQUIRE(calib2.GetCy() == cy);

  K << fx, 0, cx,
       0, fy, cy,
       0, 0, 2;
  beam_calibration::Pinhole calib3(K);
  REQUIRE(calib3.IsFull() == false);

  K << fx, 0, cx,
       0, 0, cy,
       0, 0, 1;
  beam_calibration::Pinhole calib5(K);
  REQUIRE(calib5.IsFull() == false);
}

TEST_CASE("Test distortion i/o"){
  double fx = 2338, fy = 2333, cx = 1002, cy = 784;
  beam_calibration::Pinhole calib(fx, fy, cx, cy);

  beam::Vec2 tan_coeffs;
  tan_coeffs << 1, 2;

  beam::Vec2 rad_coeffs2;
  rad_coeffs2 << 3, 4;
  beam::Vec3 rad_coeffs3;
  rad_coeffs3 << 3, 4, 5;
  beam::Vec4 rad_coeffs4;
  rad_coeffs4 << 3, 4, 5, 6;
  beam::VecX rad_coeffs6(6);
  rad_coeffs6 << 3, 4, 5, 6, 7, 8;
  beam::VecX rad_coeffs7(7);
  rad_coeffs7 << 3, 4, 5, 6, 7, 8, 9;

  calib.AddTanDist(tan_coeffs);
  REQUIRE(calib.GetTanDist() == tan_coeffs);

  REQUIRE_THROWS(calib.GetRadDist());
  calib.AddRadDist(rad_coeffs2);
  REQUIRE_THROWS(calib.GetRadDist());
  calib.AddRadDist(rad_coeffs3);
  REQUIRE(calib.GetRadDist() == rad_coeffs3);
  calib.AddRadDist(rad_coeffs4);
  REQUIRE(calib.GetRadDist() == rad_coeffs4);
  calib.AddRadDist(rad_coeffs6);
  REQUIRE(calib.GetRadDist() == rad_coeffs6);
  calib.AddRadDist(rad_coeffs7);
  REQUIRE_THROWS(calib.GetRadDist());
}

TEST_CASE("Test projection functions"){
  double fx = 2338, fy = 2333, cx = 1002, cy = 784;
  beam_calibration::Pinhole calib(fx, fy, cx, cy);

  beam::Vec2 tan_coeffs;
  tan_coeffs << 1, 2;
  beam::Vec3 rad_coeffs;
  rad_coeffs << 3, 4, 5, 6, 7, 8;

  beam::Vec3 point;
  point << 1, 2, 3;

  beam::Vec4 point_homo;
  point_homo << 1, 2, 3, 1;

  beam::Vec4 point_homo_invalid;
  point_homo_invalid << 1, 2, 3, 2;

  calib.AddTanDist(tan_coeffs);
  calib.AddRadDist(rad_coeffs);

  REQUIRE_NOTHROW(calib.ProjectPoint(point));
  REQUIRE_NOTHROW(calib.ProjectPoint(point_homo));
  REQUIRE_THROWS(calib.ProjectPoint(point_homo_invalid));
}
