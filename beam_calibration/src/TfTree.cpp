#include "beam_calibration/TfTree.h"

#include <beam_utils/log.hpp>
#include <fstream>
#include <iostream>
#include <tf2_eigen/tf2_eigen.h>

using json = nlohmann::json;

namespace beam_calibration {

void TfTree::LoadJSON(std::string& file_location) {
  BEAM_INFO("Loading file: {}", file_location.c_str());

  json J;
  int calibration_counter = 0, value_counter = 0;
  std::string type, date, method, to_frame, from_frame;
  beam::Mat4 T;
  Eigen::Affine3d TA;

  std::ifstream file(file_location);
  file >> J;

  type = J["type"];
  date = J["date"];
  method = J["method"];

  if (type != "extrinsic_calibration") {
    BEAM_CRITICAL(
        "Attempting to create TfTree with invalid json type. Type: {}",
        type.c_str());
    throw std::invalid_argument{
        "Attempting to create TfTree with invalid json type"};
    return;
  }

  SetCalibrationDate(date);

  BEAM_INFO("Type: {}", type.c_str());
  BEAM_INFO("Date: {}", date.c_str());
  BEAM_INFO("Method: {}", method.c_str());

  for (const auto& calibration : J["calibrations"]) {
    calibration_counter++;
    value_counter = 0;
    int i = 0, j = 0;

    to_frame = calibration["to_frame"];
    from_frame = calibration["from_frame"];

    for (const auto& value : calibration["transform"]) {
      value_counter++;
      T(i, j) = value.get<double>();
      if (j == 3) {
        i++;
        j = 0;
      } else {
        j++;
      }
    }
    if (value_counter != 16) {
      BEAM_CRITICAL("Invalid transform matrix in .json file.");
      throw std::invalid_argument{"Invalid transform matrix in .json file."};
      return;
    }
    TA.matrix() = T;
    this->AddTransform(TA, to_frame, from_frame);
  }
  BEAM_INFO("Saved {} transforms", calibration_counter);
}

void TfTree::AddTransform(Eigen::Affine3d& T, std::string& to_frame,
                          std::string& from_frame) {
  ros::Time time_stamp = this->start_time;
  geometry_msgs::TransformStamped T_ROS =
      EigenToROS(T, to_frame, from_frame, time_stamp);
  this->SetTransform(T_ROS, to_frame, from_frame, time_stamp, true);
}

void TfTree::AddTransform(Eigen::Affine3d& T, std::string& to_frame,
                          std::string& from_frame, ros::Time& time_stamp) {
  geometry_msgs::TransformStamped T_ROS =
      EigenToROS(T, to_frame, from_frame, time_stamp);
  this->SetTransform(T_ROS, to_frame, from_frame, time_stamp, false);
}

void TfTree::AddTransform(geometry_msgs::TransformStamped& T_ROS,
                          bool is_static) {
  std::string to_frame = T_ROS.header.frame_id;
  std::string from_frame = T_ROS.child_frame_id;
  ros::Time transform_time = T_ROS.header.stamp;
  this->SetTransform(T_ROS, to_frame, from_frame, transform_time, is_static);
}

Eigen::Affine3d TfTree::GetTransformEigen(std::string& to_frame,
                                          std::string& from_frame) {
  geometry_msgs::TransformStamped T_ROS;
  T_ROS = this->LookupTransform(to_frame, from_frame, this->start_time);
  Eigen::Affine3d T = this->ROSToEigen(T_ROS);
  return T;
}

Eigen::Affine3d TfTree::GetTransformEigen(std::string& to_frame,
                                          std::string& from_frame,
                                          ros::Time& lookup_time) {
  geometry_msgs::TransformStamped T_ROS;
  T_ROS = this->LookupTransform(to_frame, from_frame, lookup_time);
  return this->ROSToEigen(T_ROS);
}

geometry_msgs::TransformStamped
    TfTree::GetTransformROS(std::string& to_frame, std::string& from_frame,
                            ros::Time& lookup_time) {
  return this->LookupTransform(to_frame, from_frame, lookup_time);
}

geometry_msgs::TransformStamped
    TfTree::GetTransformROS(std::string& to_frame, std::string& from_frame) {
  return this->LookupTransform(to_frame, from_frame, this->start_time);
}

std::string TfTree::GetCalibrationDate() {
  if (!is_calibration_date_set_) {
    throw std::runtime_error{"cannot retrieve calibration date, value not set"};
    BEAM_CRITICAL("cannot retrieve calibration date, value not set.");
  }
  return calibration_date_;
}

void TfTree::SetCalibrationDate(std::string& calibration_date) {
  calibration_date_ = calibration_date;
  is_calibration_date_set_ = true;
}

geometry_msgs::TransformStamped TfTree::LookupTransform(std::string& to_frame,
                                                        std::string& from_frame,
                                                        ros::Time& time_stamp) {
  geometry_msgs::TransformStamped T_ROS;
  std::string transform_error;
  bool can_transform =
      Tree_.canTransform(to_frame, from_frame, time_stamp, &transform_error);

  if (can_transform) {
    T_ROS = Tree_.lookupTransform(to_frame, from_frame, time_stamp);
  } else {
    BEAM_CRITICAL(
        "Cannot look up transform from frame {} to {}. Transform Error "
        "Message: {}",
        from_frame.c_str(), to_frame.c_str(), transform_error.c_str());
    throw std::runtime_error{"Cannot look up transform."};
  }
  return T_ROS;
}

geometry_msgs::TransformStamped TfTree::EigenToROS(Eigen::Affine3d& T,
                                                   std::string& to_frame,
                                                   std::string& from_frame,
                                                   ros::Time& time_stamp) {
  if (!beam::IsTransformationMatrix(T.matrix())) {
    BEAM_CRITICAL("Invalid transformation matrix input");
    throw std::runtime_error{"Invalid transformation matrix"};
  }
  geometry_msgs::TransformStamped T_ROS = tf2::eigenToTransform(T);
  T_ROS.header.seq = 1;
  T_ROS.header.frame_id = to_frame;
  T_ROS.child_frame_id = from_frame;
  T_ROS.header.stamp = time_stamp;
  return T_ROS;
}

Eigen::Affine3d TfTree::ROSToEigen(geometry_msgs::TransformStamped T_ROS) {
  return tf2::transformToEigen(T_ROS);
}

void TfTree::SetTransform(geometry_msgs::TransformStamped& T_ROS,
                          std::string& to_frame, std::string& from_frame,
                          ros::Time& time_stamp, bool is_static) {
  /* ---------------------------------------------------------------------------
  here's the logic:
  if static and exact transform from child to parent exists, output error
  if static and a child already has a parent, add the inverse
  if static and child doesn't already have a parent, add normally
  if dynamic and adding identical child-parent then add normally
  if dynamic and adding child which already has a parent, add inverse
  if both frames have a parent already then output error
  ----------------------------------------------------------------------------*/

  // Static case:
  std::string transform_error;
  if (is_static) {
    // Check case the exact transform exists
    bool transform_exists =
        Tree_.canTransform(to_frame, from_frame, time_stamp, &transform_error);
    if (transform_exists) {
      BEAM_CRITICAL("Trying to add a static transform that already exists "
                    "(to_frame: {}, from frame {})",
                    to_frame.c_str(), from_frame.c_str());
      throw std::runtime_error{
          "Cannot add transform. Transform already exists."};
    }

    // Check for case where a child frame already has a parent
    std::string parent;
    bool parent_exists = Tree_._getParent(from_frame, time_stamp, parent);
    if (parent_exists) {
      // Then add inverse
      BEAM_INFO(
          "Attemping to add transform from {} to {}, but frame {} already "
          "has a parent ({}). Adding inverse of inputted transform.",
          from_frame.c_str(), to_frame.c_str(), from_frame.c_str(),
          parent.c_str());
      tf2::Transform inverse_transform;
      tf2::fromMsg(T_ROS.transform, inverse_transform);
      inverse_transform = inverse_transform.inverse();
      T_ROS.transform = tf2::toMsg(inverse_transform);
      T_ROS.header.frame_id = from_frame;
      T_ROS.child_frame_id = to_frame;

      if (!Tree_.setTransform(T_ROS, "TfTree", is_static)) {
        BEAM_CRITICAL("Cannot add transform from {} to {}", from_frame.c_str(),
                      to_frame.c_str());
        throw std::runtime_error{"Cannot add transform."};
      }
      this->InsertFrame(from_frame, to_frame);
      return;
    } else {
      // Add transform normally
      if (!Tree_.setTransform(T_ROS, "TfTree", is_static)) {
        BEAM_CRITICAL("Cannot add transform from {} to {}", from_frame.c_str(),
                      to_frame.c_str());
        throw std::runtime_error{"Cannot add transform."};
      }
      this->InsertFrame(to_frame, from_frame);
      return;
    }
  }

  // Dynamic transform case
  std::string parent;
  bool parent_exists = Tree_._getParent(from_frame, time_stamp, parent);
  if (parent_exists && parent == to_frame) {
    // add normally
    if (!Tree_.setTransform(T_ROS, "TfTree", is_static)) {
      BEAM_CRITICAL("Cannot add transform from {} to {}", from_frame.c_str(),
                    to_frame.c_str());
      throw std::runtime_error{"Cannot add transform."};
    }
    this->InsertFrame(to_frame, from_frame);
    return;
  } else if (parent_exists) {
    // add inverse only if "new" child doesn't already have another parent
    parent_exists = Tree_._getParent(to_frame, time_stamp, parent);
    if (parent_exists && parent != from_frame) {
      BEAM_CRITICAL("Cannot add transform from {} to {}", from_frame.c_str(),
                    to_frame.c_str());
      throw std::runtime_error{"Cannot add transform."};
    } else {
      // add inverse
      // BEAM_INFO("Attemping to add transform from {} to {}, but frame {}
      // already "
      //   "has a parent ({}). Adding inverse of inputted transform.",
      //   from_frame.c_str(), to_frame.c_str(), to_frame.c_str(),
      //   parent.c_str());
      tf2::Transform inverse_transform;
      tf2::fromMsg(T_ROS.transform, inverse_transform);
      inverse_transform = inverse_transform.inverse();
      T_ROS.transform = tf2::toMsg(inverse_transform);
      T_ROS.header.frame_id = from_frame;
      T_ROS.child_frame_id = to_frame;

      if (!Tree_.setTransform(T_ROS, "TfTree", is_static)) {
        BEAM_CRITICAL("Cannot add transform from {} to {}", from_frame.c_str(),
                      to_frame.c_str());
        throw std::runtime_error{"Cannot add transform."};
      }
      this->InsertFrame(from_frame, to_frame);
      return;
    }
  } else {
    // add normally
    if (!Tree_.setTransform(T_ROS, "TfTree", is_static)) {
      BEAM_CRITICAL("Cannot add transform from {} to {}", from_frame.c_str(),
                    to_frame.c_str());
      throw std::runtime_error{"Cannot add transform."};
    }
    this->InsertFrame(to_frame, from_frame);
    return;
  }
}

void TfTree::InsertFrame(std::string& to_frame, std::string& from_frame) {
  auto it = frames_.find(from_frame);
  if (it == frames_.end()) {
    // from_frame not added yet
    frames_.emplace(from_frame, std::vector<std::string>{to_frame});
  } else {
    for (auto child_frame : it->second) {
      // transform already exists. Return.
      if (child_frame == to_frame) return;
    }
    // from_frame already exists in the map, insert to_frame at the back
    frames_[from_frame].push_back(to_frame);
  }
}

} // namespace beam_calibration
