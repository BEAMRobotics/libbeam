#include "beam/calibration/TFTree.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <beam/utils/math.hpp>
#include <tf2/buffer_core.h>

namespace beam_calibration {

  void TFTree::AddTransform(beam::Mat4 Tnew, std::string from_frame,
                            std::string to_frame, std::string calib_date){

    Eigen::Affine3d TAnew;
    TAnew.matrix() = Tnew;

    geometry_msgs::TransformStamped T = tf2::eigenToTransform(TAnew);
    // T.header
    // Tree_.setTransform();
  }


} // namespace beam_calibration
