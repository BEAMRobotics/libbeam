#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include "beam/utils/math.hpp"
#include "beam/calibration/TfTree.h"


TEST_CASE("Test Tree building and retrieving"){
  beam_calibration::TfTree Tree;
  beam::Mat4 T_BASELINK_HVLP, T_X1_HVLP;
  Eigen::Affine3d TA_BASELINK_HVLP, TA_X1_HVLP, TA_BASELINK_X1_calc,
                  TA_BASELINK_X1_lookup;

  T_BASELINK_HVLP << 1.00000,   0.00000,   0.00000,   0.2100,
                     0.00000,   1.00000,   0.00000,   0.00000,
                     0.00000,   0.00000,   1.00000,   0.35200,
                     0.00000,   0.00000,   0.00000,   1.00000;

  T_X1_HVLP << 0.00000,   0.00000,  -1.00000,  -0.0800,
               0.00000,   1.00000,   0.00000,   0.00000,
               1.00000,   0.00000,   0.00000,   -0.0400,
               0.00000,   0.00000,   0.00000,   1.00000;

  TA_BASELINK_HVLP.matrix() = T_BASELINK_HVLP;
  TA_X1_HVLP.matrix() = T_X1_HVLP;

  std::string to_frame = "BASELINK";
  std::string from_frame = "HVLP";
  std::string calib_date = "2018_12_20";
  Tree.AddTransform(TA_BASELINK_HVLP, to_frame, from_frame, calib_date);
  to_frame = "X1";
  from_frame = "HVLP";
  calib_date = "2018_12_20";
  Tree.AddTransform(TA_X1_HVLP, to_frame, from_frame, calib_date);

  to_frame = "BASELINK";
  from_frame = "X1";
  TA_BASELINK_X1_calc = TA_BASELINK_HVLP * TA_X1_HVLP.inverse();
  TA_BASELINK_X1_lookup = Tree.GetTransform(to_frame, from_frame);
  REQUIRE(TA_BASELINK_X1_calc.matrix() == TA_BASELINK_X1_lookup.matrix());
  REQUIRE(Tree.GetCalibrationDate() == calib_date);
}
