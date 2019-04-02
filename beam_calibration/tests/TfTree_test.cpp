#define CATCH_CONFIG_MAIN
#include "beam/calibration/TfTree.h"
#include "beam/utils/math.hpp"
#include <catch2/catch.hpp>

TEST_CASE("Test Tree building and retrieving") {
  beam_calibration::TfTree Tree;
  beam::Mat4 T_BASELINK_HVLP, T_X1_HVLP;
  Eigen::Affine3d TA_BASELINK_HVLP, TA_X1_HVLP, TA_BASELINK_X1_calc,
      TA_BASELINK_X1_lookup;

  T_BASELINK_HVLP << 1.00000, 0.00000, 0.00000, 0.2100, 0.00000, 1.00000,
      0.00000, 0.00000, 0.00000, 0.00000, 1.00000, 0.35200, 0.00000, 0.00000,
      0.00000, 1.00000;

  T_X1_HVLP << 0.00000, 0.00000, -1.00000, -0.0800, 0.00000, 1.00000, 0.00000,
      0.00000, 1.00000, 0.00000, 0.00000, -0.0400, 0.00000, 0.00000, 0.00000,
      1.00000;

  TA_BASELINK_HVLP.matrix() = T_BASELINK_HVLP;
  TA_X1_HVLP.matrix() = T_X1_HVLP;

  std::string to_frame1 = "BASELINK";
  std::string from_frame1 = "HVLP";
  std::string calib_date = "2018_12_20";
  Tree.AddTransform(TA_BASELINK_HVLP, to_frame1, from_frame1, calib_date);
  std::string to_frame2 = "X1";
  std::string from_frame2 = "HVLP";
  Tree.AddTransform(TA_X1_HVLP, to_frame2, from_frame2, calib_date);

  std::string to_frame3 = "BASELINK";
  std::string from_frame3 = "X1";
  TA_BASELINK_X1_calc = TA_BASELINK_HVLP * TA_X1_HVLP.inverse();
  TA_BASELINK_X1_lookup = Tree.GetTransform(to_frame3, from_frame3);
  int round_precision = 100000;
  REQUIRE(TA_BASELINK_HVLP.matrix() ==
          beam::RoundMatrix(Tree.GetTransform(to_frame1, from_frame1).matrix(),
                            round_precision));
  REQUIRE(TA_X1_HVLP.matrix() ==
          beam::RoundMatrix(Tree.GetTransform(to_frame2, from_frame2).matrix(),
                            round_precision));
  REQUIRE(Tree.GetCalibrationDate() == calib_date);
  REQUIRE_THROWS(Tree.AddTransform(TA_BASELINK_X1_calc, to_frame3, from_frame3,
                                   calib_date));
}
