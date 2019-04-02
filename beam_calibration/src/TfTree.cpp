#include "beam/calibration/TfTree.h"
#include <beam/utils/log.hpp>
#include <beam/utils/math.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>

namespace beam_calibration {

void TfTree::AddTransform(Eigen::Affine3d& TAnew, std::string& to_frame,
                          std::string& from_frame, std::string& calib_date) {
  ros::Time time0(0);
  std::string *transform_error(new std::string);
  bool transform_exists =
      Tree_.canTransform(to_frame, from_frame, time0, transform_error);
  if (transform_exists) {
    throw std::runtime_error{"Cannot add transform. Transform already exists."};
    LOG_ERROR("Cannot add transform from frame %s to %s. Frame already exists",
              from_frame.c_str(), to_frame.c_str());
  } else {
    geometry_msgs::TransformStamped T = tf2::eigenToTransform(TAnew.inverse());
    T.header.seq = 1;
    T.header.frame_id = from_frame;
    T.child_frame_id = to_frame;
    SetCalibrationDate(calib_date);
    bool transform_valid = Tree_.setTransform(T, "TfTree", true);
    if (!transform_valid) {
      throw std::runtime_error{"Cannot add transform. Transform invalid."};
      LOG_ERROR("Cannot add transform from frame %s to %s. Transform invalid",
                from_frame.c_str(), to_frame.c_str());
    }
  }
}

Eigen::Affine3d TfTree::GetTransform(std::string& to_frame,
                                     std::string& from_frame) {
  Eigen::Affine3d TA_target_source;
  std::string *transform_error(new std::string);
  ros::Time time0(0);
  bool can_transform =
      Tree_.canTransform(to_frame, from_frame, time0, transform_error);

  if (can_transform) {
    geometry_msgs::TransformStamped T_target_source;
    T_target_source = Tree_.lookupTransform(to_frame, from_frame, time0);
    TA_target_source = tf2::transformToEigen(T_target_source);
  } else {
    throw std::runtime_error{"Cannot look up transform."};
    LOG_ERROR("Cannot look up transform from frame %s to %s. Transform Error "
              "Message: %s",
              from_frame.c_str(), to_frame.c_str(), transform_error->c_str());
  }
  return TA_target_source;
}

void TfTree::SetCalibrationDate(std::string& calibraiton_date) {
  calibraiton_date_ = calibraiton_date;
  is_calibration_date_set_ = true;
}

std::string TfTree::GetCalibrationDate() {
  if (!is_calibration_date_set_) {
    throw std::runtime_error{"cannot retrieve calibration date, value not set"};
    LOG_ERROR("cannot retrieve calibration date, value not set.");
  }
  return calibraiton_date_;
}

} // namespace beam_calibration
