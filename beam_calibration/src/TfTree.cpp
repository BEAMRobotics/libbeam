#include "beam/calibration/TfTree.h"
#include <beam/utils/log.hpp>
#include <beam/utils/math.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>

using json = nlohmann::json;

namespace beam_calibration {

void TfTree::LoadJSON(std::string &file_location) {
  LOG_INFO("Loading file: %s", file_location.c_str());

  json J;
  int transform_counter = 0, value_counter = 0;
  std::string type, date, method, to_frame, from_frame;
  beam::Mat4 T;
  Eigen::Affine3d TA;

  std::ifstream file(file_location);
  file >> J;

  type = J["type"];
  date = J["date"];
  method = J["method"];
  SetCalibrationDate(date);

  LOG_INFO("Type: %s", type.c_str());
  LOG_INFO("Date: %s", date.c_str());
  LOG_INFO("Method: %s", method.c_str());

  for (const auto& transform : J["transforms"]){
    transform_counter++;
    value_counter = 0;
    int i = 0, j = 0;

    to_frame = transform["to_frame"];
    from_frame = transform["from_frame"];

    for (const auto& value : transform["transform"]){
      value_counter++;
      T(i,j) = value.get<double>();
      if(j==3){
        i++;
        j=0;
      } else {
        j++;
      }
    }
    if(value_counter != 16){
      LOG_ERROR("Invalid transform matrix in .json file.");
      throw std::runtime_error{"Invalid transform matrix in .json file."};
      return;
    }
    TA.matrix() = T;
    AddTransform(TA, to_frame, from_frame);
  }
  LOG_INFO("Saved %d transforms", transform_counter);
}

void TfTree::AddTransform(Eigen::Affine3d& TAnew, std::string& to_frame,
                          std::string& from_frame) {
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

void TfTree::SetCalibrationDate(std::string& calibration_date) {
  calibration_date_ = calibration_date;
  is_calibration_date_set_ = true;
}

std::string TfTree::GetCalibrationDate() {
  if (!is_calibration_date_set_) {
    throw std::runtime_error{"cannot retrieve calibration date, value not set"};
    LOG_ERROR("cannot retrieve calibration date, value not set.");
  }
  return calibration_date_;
}

} // namespace beam_calibration
