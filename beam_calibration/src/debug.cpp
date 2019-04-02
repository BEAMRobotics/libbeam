#include "beam/calibration/TfTree.h"
#include "beam/utils/math.hpp"

int main() {
  beam_calibration::TfTree Tree;
  beam::Mat4 T_BASELINK_HVLP, T_X1_HVLP;
  Eigen::Affine3d TA_BASELINK_HVLP, TA_X1_HVLP, TA_BASELINK_X1_calc,
      TA_BASELINK_X1_lookup, TA_BASELINK_HVLP2, TA_X1_HVLP2;

  T_BASELINK_HVLP << 1.00000, 0.00000, 0.00000, 0.2100, 0.00000, 1.00000,
      0.00000, 0.00000, 0.00000, 0.00000, 1.00000, 0.35200, 0.00000, 0.00000,
      0.00000, 1.00000;

  T_X1_HVLP << 0.00000, 0.00000, -1.00000, -0.0800, 0.00000, 1.00000, 0.00000,
      0.00000, 1.00000, 0.00000, 0.00000, -0.0400, 0.00000, 0.00000, 0.00000,
      1.00000;

  // T_X1_HVLP << 0.00000,   0.00000,  -1.00000,  -0.0801,
  //              0.00000,   1.00000,   0.00000,   0.08058,
  //              1.00000,   0.00000,   0.00000,   -0.08614,
  //              0.00000,   0.00000,   0.00000,   0.08414;

  TA_BASELINK_HVLP.matrix() = T_BASELINK_HVLP;
  TA_X1_HVLP.matrix() = T_X1_HVLP;

  std::string to_frame1 = "X1";
  std::string from_frame1 = "HVLP";
  std::string calib_date = "2018_12_20";
  Tree.AddTransform(TA_X1_HVLP, to_frame1, from_frame1, calib_date);
  TA_X1_HVLP2 = Tree.GetTransform(to_frame1, from_frame1);
  beam::Mat4 T_X1_HVLP2 = TA_X1_HVLP2.matrix();
  std::cout << "T_X1_HVLP: \n" << TA_X1_HVLP.matrix() << "\n";
  std::cout << "T_X1_HVLP2: \n" << TA_X1_HVLP2.matrix() << "\n";
  std::cout << "TA_X1_HVLP.matrix().inverse(): \n" << TA_X1_HVLP.matrix().inverse() << "\n";

  //
  // std::string to_frame2 = "X1";
  // std::string from_frame2 = "HVLP";
  // Tree.AddTransform(TA_X1_HVLP, to_frame2, from_frame2, calib_date);
  //
  // std::string to_frame3 = "BASELINK";
  // std::string from_frame3 = "X1";
  // TA_BASELINK_X1_calc = TA_BASELINK_HVLP * TA_X1_HVLP.inverse();
  // TA_BASELINK_X1_lookup = Tree.GetTransform(to_frame3, from_frame3);
}
