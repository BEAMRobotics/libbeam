#include "beam/calibration/TfTree.h"
#include "beam/calibration/Pinhole.h"
#include "beam/utils/math.hpp"
#include <boost/filesystem.hpp>

int main() {
  /*
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
  */

  /*
  beam_calibration::TfTree Tree;
  beam::Mat4 T_BASELINK_HVLP, T_X1_HVLP;
  Eigen::Affine3d TA_BASELINK_HVLP, TA_X1_HVLP,
                  TA_BASELINK_HVLP_JSON, TA_X1_HVLP_JSON;

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

  // Load Tree from json
  std::string filename = "extrinsics.json";
  std::string file_location = __FILE__;
  file_location.erase(file_location.end() - 13, file_location.end());
  file_location += "tests/test_data/";
  file_location += filename;
  Tree.LoadJSON(file_location);
  */

  beam_calibration::Pinhole F1;
  std::string filename = "F1.json";
  std::string file_location = __FILE__;
  file_location.erase(file_location.end() - 13, file_location.end());
  file_location += "tests/test_data/";
  file_location += filename;
  F1.LoadJSON(file_location);
}
