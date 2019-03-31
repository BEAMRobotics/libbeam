#include "beam/calibration/TfTree.h"
#include <beam/utils/log.hpp>
#include <beam/utils/math.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>

namespace beam_calibration {

void TfTree::AddTransform(Eigen::Affine3d &TAnew, std::string &from_frame,
                          std::string &to_frame, std::string &calib_date) {
  geometry_msgs::TransformStamped T = tf2::eigenToTransform(TAnew);
  T.header.seq = 1;
  T.header.frame_id = from_frame;
  T.child_frame_id = to_frame;

  bool transform_valid = Tree_.setTransform(T, "TfTree", true);
  if(!transform_valid){
    throw std::runtime_error{"Cannot add transform."};
    LOG_ERROR("Cannot add transform from frame %s to %s.", from_frame.c_str(),
              to_frame.c_str());
  }
}

Eigen::Affine3d TfTree::GetTransform(std::string &from_frame,
                                     std::string &to_frame) {
  std::string *transform_error;
  ros::Time time0(0);
  bool can_transform =
      Tree_.canTransform(to_frame, from_frame, time0, transform_error);

  if (can_transform) {
    geometry_msgs::TransformStamped T_target_source;
    T_target_source = Tree_.lookupTransform(to_frame, from_frame, time0);
  } else {
    throw std::runtime_error{"Cannot look up transform."};
    LOG_ERROR("Cannot look up transform from frame %s to %s. Transform Error "
              "Message: %s", from_frame.c_str(), to_frame.c_str(),
              transform_error->c_str());
  }
}

void TfTree::SetCalibrationDate(std::string &calibraiton_date){
  calibraiton_date_ = calibraiton_date;
  is_calibration_date_set_ = true;
}

std::string TfTree::GetCalibrationDate(){
  if(!is_calibration_date_set_){
    throw std::runtime_error{"cannot retrieve calibration date, value not set"};
    LOG_ERROR("cannot retrieve calibration date, value not set.");
  }
  return calibraiton_date_;
}

} // namespace beam_calibration
