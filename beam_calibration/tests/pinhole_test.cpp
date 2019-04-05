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
  REQUIRE(calib.IsKFull() == true);
  REQUIRE(calib.GetK() == K);
  REQUIRE(calib.GetFx() == fx);
  REQUIRE(calib.GetFy() == fy);
  REQUIRE(calib.GetCx() == cx);
  REQUIRE(calib.GetCy() == cy);

  REQUIRE(calib2.GetK() == K);
  REQUIRE(calib2.IsKFull() == true);
  REQUIRE(calib2.GetFx() == fx);
  REQUIRE(calib2.GetFy() == fy);
  REQUIRE(calib2.GetCx() == cx);
  REQUIRE(calib2.GetCy() == cy);

  K << fx, 0, cx,
       0, fy, cy,
       0, 0, 2;
  beam_calibration::Pinhole calib3(K);
  REQUIRE(calib3.IsKFull() == false);

  K << fx, 0, cx,
       0, 0, cy,
       0, 0, 1;
  beam_calibration::Pinhole calib5(K);
  REQUIRE(calib5.IsKFull() == false);
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

  calib.SetTanDist(tan_coeffs);
  REQUIRE(calib.GetTanDist() == tan_coeffs);

  REQUIRE_THROWS(calib.GetRadDist());
  calib.SetRadDist(rad_coeffs2);
  REQUIRE_THROWS(calib.GetRadDist());
  calib.SetRadDist(rad_coeffs3);
  REQUIRE(calib.GetRadDist() == rad_coeffs3);
  calib.SetRadDist(rad_coeffs4);
  REQUIRE(calib.GetRadDist() == rad_coeffs4);
  calib.SetRadDist(rad_coeffs6);
  REQUIRE(calib.GetRadDist() == rad_coeffs6);
  calib.SetRadDist(rad_coeffs7);
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

  calib.SetTanDist(tan_coeffs);
  calib.SetRadDist(rad_coeffs);

  REQUIRE_NOTHROW(calib.ProjectPoint(point));
  REQUIRE_NOTHROW(calib.ProjectPoint(point_homo));
  REQUIRE_THROWS(calib.ProjectPoint(point_homo_invalid));
}

TEST_CASE("Testing LoadJSON function"){
  beam_calibration::Pinhole F1;
  int round_precision = 10000000;

  std::string filename = "F1.json";
  std::string file_location = __FILE__;
  file_location.erase(file_location.end() - 16, file_location.end());
  file_location += "test_data/";
  file_location += filename;
  F1.LoadJSON(file_location);

  beam::Mat3 K;
  beam::Vec2 tan_coeffs, img_dims;
  beam::Vec3 rad_coeffs;

  std::string date = "2018_12_20";
  K << 2338.485520924695, 0,  1002.8381839138167,
       0, 2333.0927287230647, 784.1498440053573,
       0, 0, 1;
  tan_coeffs << -0.0005326294604360527, -0.0004378797791316729;
  rad_coeffs << -0.2294924671994032, 0.18008566892263364, 0;
  img_dims << 2048, 1536;

  REQUIRE(date == F1.GetCalibrationDate());
  REQUIRE(beam::RoundMatrix(K, round_precision) ==
          beam::RoundMatrix(F1.GetK(), round_precision));
  REQUIRE(beam::RoundMatrix(tan_coeffs, round_precision) ==
          beam::RoundMatrix(F1.GetTanDist(), round_precision));
  REQUIRE(beam::RoundMatrix(img_dims, round_precision) ==
          beam::RoundMatrix(F1.GetImgDims(), round_precision));
}
