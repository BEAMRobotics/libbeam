#include "beam/utils/math.hpp"
#include "beam/calibration/TfTree.h"

int main(){
  beam_calibration::TfTree Tree;
  beam::Mat4 T;
  T.setIdentity();
  Eigen::Affine3d TA;
  TA.matrix() = T;
  std::string from_frame = "base_link";
  std::string to_frame = "hvlp_link";
  std::string calib_date = "2019_01_01";
  Tree.AddTransform(TA, from_frame, to_frame, calib_date);

}
