#define CATCH_CONFIG_MAIN
#include "beam_calibration/TfTree.h"
#include "beam_utils/math.hpp"
#include <catch2/catch.hpp>
#include <boost/filesystem.hpp>

TEST_CASE("Test Tree building and retrieving") {
  beam_calibration::TfTree Tree;
  beam::Mat4 T_BASELINK_HVLP, T_X1_HVLP;
  Eigen::Affine3d TA_BASELINK_HVLP, TA_X1_HVLP, TA_BASELINK_X1_calc,
      TA_BASELINK_X1_lookup;

  T_BASELINK_HVLP << 1.00000, 0.00000, 0.00000, 0.2100,
                     0.00000, 1.00000, 0.00000, 0.00000,
                     0.00000, 0.00000, 1.00000, 0.35200,
                     0.00000, 0.00000, 0.00000, 1.00000;

  T_X1_HVLP << 0.00000, 0.00000, -1.00000, -0.0800,
               0.00000, 1.00000, 0.00000, 0.00000,
               1.00000, 0.00000, 0.00000, -0.0400,
               0.00000, 0.00000, 0.00000,  1.00000;

  TA_BASELINK_HVLP.matrix() = T_BASELINK_HVLP;
  TA_X1_HVLP.matrix() = T_X1_HVLP;

  std::string to_frame1 = "base_link";
  std::string from_frame1 = "hvlp_link";
  std::string calib_date = "2018_12_20";
  Tree.AddTransform(TA_BASELINK_HVLP, to_frame1, from_frame1);
  std::string to_frame2 = "X1_link";
  std::string from_frame2 = "hvlp_link";
  Tree.AddTransform(TA_X1_HVLP, to_frame2, from_frame2);
  Tree.SetCalibrationDate(calib_date);

  std::string to_frame3 = "base_link";
  std::string from_frame3 = "X1_link";
  TA_BASELINK_X1_calc.matrix() = TA_BASELINK_HVLP.matrix() * TA_X1_HVLP.matrix().inverse();
  TA_BASELINK_X1_lookup = Tree.GetTransform(to_frame3, from_frame3);
  int round_precision = 10000000;
  REQUIRE(TA_X1_HVLP.matrix() ==
          beam::RoundMatrix(Tree.GetTransform(to_frame2, from_frame2).matrix(),
                            round_precision));
  REQUIRE(TA_BASELINK_HVLP.matrix() ==
          beam::RoundMatrix(Tree.GetTransform(to_frame1, from_frame1).matrix(),
                            round_precision));
  REQUIRE(beam::RoundMatrix(TA_BASELINK_X1_calc.matrix(), round_precision) ==
          beam::RoundMatrix(TA_BASELINK_X1_lookup.matrix(), round_precision));
  REQUIRE(Tree.GetCalibrationDate() == calib_date);
  REQUIRE_THROWS(Tree.AddTransform(TA_BASELINK_X1_calc, to_frame3, from_frame3));
}

TEST_CASE("Test loading tree from .json"){
  // create empty objects
  beam_calibration::TfTree Tree;
  beam::Mat4 T_BASELINK_HVLP, T_X1_HVLP, T_BASELINK_HVLP_JSON, T_X1_HVLP_JSON,
             T_BASELINK_X1, T_BASELINK_X1_JSON;
  int round_precision = 10000000;

  // add true data
  std::string calib_date = "2018_12_20";
  T_BASELINK_HVLP << 1.00000, 0.00000, 0.00000, 0.2100,
                     0.00000, 1.00000, 0.00000, 0.00000,
                     0.00000, 0.00000, 1.00000, 0.35200,
                     0.00000, 0.00000, 0.00000, 1.00000;

  T_X1_HVLP << 0.00000, 0.00000, -1.00000, -0.0800,
               0.00000, 1.00000, 0.00000, 0.00000,
               1.00000, 0.00000, 0.00000, -0.0400,
               0.00000, 0.00000, 0.00000,  1.00000;

  T_BASELINK_X1 = T_BASELINK_HVLP * T_X1_HVLP.inverse();

  // Load Tree from json
  std::string filename = "extrinsics.json";
  std::string file_location = __FILE__;
  file_location.erase(file_location.end() - 15, file_location.end());
  file_location += "test_data/";
  file_location += filename;
  Tree.LoadJSON(file_location);

  // look up transforms needed for tests
  std::string to_frame1 = "base_link";
  std::string from_frame1 = "hvlp_link";
  std::string to_frame2 = "X1_link";
  std::string from_frame2 = "hvlp_link";
  std::string to_frame3 = "base_link";
  std::string from_frame3 = "X1_link";

  T_BASELINK_HVLP_JSON = Tree.GetTransform(to_frame1, from_frame1).matrix();
  T_X1_HVLP_JSON = Tree.GetTransform(to_frame2, from_frame2).matrix();
  T_BASELINK_X1_JSON = Tree.GetTransform(to_frame3, from_frame3).matrix();

  // perform tests
  REQUIRE(Tree.GetCalibrationDate() == calib_date);
  REQUIRE(beam::RoundMatrix(T_BASELINK_HVLP, round_precision) ==
          beam::RoundMatrix(T_BASELINK_HVLP_JSON, round_precision));
  REQUIRE(beam::RoundMatrix(T_X1_HVLP, round_precision) ==
          beam::RoundMatrix(T_X1_HVLP_JSON, round_precision));
  REQUIRE(beam::RoundMatrix(T_BASELINK_X1, round_precision) ==
          beam::RoundMatrix(T_BASELINK_X1_JSON, round_precision));
}

TEST_CASE("Testing multiple parent case"){
  beam_calibration::TfTree Tree1, Tree2;
  beam::Mat4 T_HVLP_BASELINK, T_X1_HVLP, T_X1_IMU1;
  Eigen::Affine3d TA_HVLP_BASELINK, TA_X1_HVLP, TA_X1_IMU1, TA_IMU1_X1, T;

  T_HVLP_BASELINK << 1.00000, 0.00000, 0.00000, -0.2100,
                     0.00000, 1.00000, 0.00000, 0.00000,
                     0.00000, 0.00000, 1.00000, -0.35200,
                     0.00000, 0.00000, 0.00000, 1.00000;

  T_X1_HVLP << 0.00000, 0.00000, -1.00000, -0.0800,
               0.00000, 1.00000, 0.00000, 0.00000,
               1.00000, 0.00000, 0.00000, -0.0400,
               0.00000, 0.00000, 0.00000,  1.00000;

  T_X1_IMU1 << 0.00000,   0.00000,  -1.00000,  0.00000,
              -1.00000,   0.00000,   0.00000,  0.00000,
               0.00000,   1.00000,   0.00000,  -0.0900,
               0.00000,   0.00000,   0.00000,  1.00000;

   TA_HVLP_BASELINK.matrix() = T_HVLP_BASELINK;
   TA_X1_HVLP.matrix() = T_X1_HVLP;
   TA_X1_IMU1.matrix() = T_X1_IMU1;
   TA_IMU1_X1 = TA_X1_IMU1.inverse();
   std::string to_frame1 = "hvlp_link";
   std::string from_frame1 = "base_link";
   std::string to_frame2 = "X1_link";
   std::string from_frame2 = "hvlp_link";
   std::string to_frame3 = "X1_link";
   std::string from_frame3 = "IMU1_link";

   Tree1.AddTransform(TA_HVLP_BASELINK, to_frame1, from_frame1);
   Tree1.AddTransform(TA_X1_HVLP, to_frame2, from_frame2);
   REQUIRE_NOTHROW(T = Tree1.GetTransform(to_frame2, from_frame1));
   Tree1.AddTransform(TA_X1_IMU1, to_frame3, from_frame3);
   REQUIRE_NOTHROW(T = Tree1.GetTransform(to_frame2, from_frame1));
   //
   // Tree2.AddTransform(TA_HVLP_BASELINK, to_frame1, from_frame1);
   // Tree2.AddTransform(TA_X1_HVLP, to_frame2, from_frame2);
   // REQUIRE_NOTHROW(T = Tree2.GetTransform(to_frame2, from_frame1));
   // Tree2.AddTransform(TA_IMU1_X1, from_frame3, to_frame3);
   // REQUIRE_NOTHROW(T = Tree2.GetTransform(to_frame2, from_frame1));
   // REQUIRE_NOTHROW(T = Tree2.GetTransform(from_frame1, from_frame3));
}
