#include "beam_mapping/Poses.h"

#include "beam_utils/log.hpp"
#include "beam_utils/math.hpp"

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

namespace beam_mapping {

void Poses::AddBagName(const std::string& bag_name) {
  bag_name_ = bag_name;
}

std::string Poses::GetBagName() {
  return bag_name_;
}

void Poses::AddPoseFileDate(const std::string& pose_file_date) {
  pose_file_date_ = pose_file_date;
}

std::string Poses::GetPoseFileDate() {
  return pose_file_date_;
}

void Poses::AddFixedFrame(const std::string& fixed_frame) {
  fixed_frame_ = fixed_frame;
}

std::string Poses::GetFixedFrame() {
  return fixed_frame_;
}

void Poses::AddMovingFrame(const std::string& moving_frame) {
  moving_frame_ = moving_frame;
}

std::string Poses::GetMovingFrame() {
  return moving_frame_;
}

void Poses::AddTimeStamps(const std::vector<ros::Time>& time_stamps) {
  time_stamps_ = time_stamps;
}

std::vector<ros::Time> Poses::GetTimeStamps() {
  return time_stamps_;
}

void Poses::AddSingleTimeStamp(const ros::Time& time_stamp) {
  time_stamps_.push_back(time_stamp);
}

void Poses::AddPoses(
    const std::vector<Eigen::Affine3d,
                      Eigen::aligned_allocator<Eigen::Affine3d>>& poses) {
  poses_ = poses;
}

std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>
    Poses::GetPoses() {
  return poses_;
}

void Poses::AddSinglePose(const Eigen::Affine3d& pose) {
  poses_.push_back(pose);
}

void Poses::WriteToPoseFile(const std::string output_dir) {
  if (poses_.size() != time_stamps_.size()) {
    BEAM_CRITICAL("Number of time stamps not equal to number of poses. Not "
                  "outputting to pose file.");
    throw std::runtime_error{"Number of time stamps not equal to number of "
                             "poses. Cannot create pose "
                             "file."};
  }

  if (!boost::filesystem::is_directory(output_dir)) {
    BEAM_INFO("Output directory does not exist, creating now.");
    boost::filesystem::create_directories(output_dir);
  }

  // write to json
  std::string J_string, J_poses_string, J_pose_k_string;
  nlohmann::json J, J_pose_k;
  J = {{"bag_name", bag_name_},
       {"pose_file_date", pose_file_date_},
       {"fixed_frame", fixed_frame_},
       {"moving_frame", moving_frame_}};

  J_string = J.dump();
  J_poses_string = "{\"poses\": []}";
  for (uint64_t k = 0; k < poses_.size(); k++) {
    beam::Mat4 Tk = poses_[k].matrix();
    J_pose_k = {{"time_stamp_sec", time_stamps_[k].sec},
                {"time_stamp_nsec", time_stamps_[k].nsec},
                {"transform", {Tk(0, 0), Tk(0, 1), Tk(0, 2), Tk(0, 3),
                               Tk(1, 0), Tk(1, 1), Tk(1, 2), Tk(1, 3),
                               Tk(2, 0), Tk(2, 1), Tk(2, 2), Tk(2, 3),
                               Tk(3, 0), Tk(3, 1), Tk(3, 2), Tk(3, 3)}}};
    J_pose_k_string = J_pose_k.dump();
    J_poses_string.erase(J_poses_string.end()-2, J_poses_string.end()); // erase end ]}
    if(k>0){
      J_poses_string = J_poses_string + ", ";
    }
    J_poses_string = J_poses_string + J_pose_k_string + "]}"; // add new pose
  }

  J_string.erase(J_string.end()-1, J_string.end()); // erase end }
  J_poses_string.erase(0,1); // erase start {
  J_string = J_string + "," + J_poses_string; // add poses
  J = nlohmann::json::parse(J_string);

  std::string output_file = output_dir + pose_file_date_ + "_poses.json";
  BEAM_INFO("Saving poses to file: {}", output_file.c_str());
  std::ofstream filejson(output_file);
  filejson << std::setw(4) << J << std::endl;
}

void Poses::LoadPoseFile(const std::string input_pose_file_path) {
  BEAM_INFO("Loading pose file: {}", input_pose_file_path.c_str());
  nlohmann::json J;
  std::ifstream file(input_pose_file_path);
  file >> J;
  bag_name_ = J["bag_name"];
  fixed_frame_ = J["fixed_frame"];
  moving_frame_ = J["moving_frame"];
  pose_file_date_ = J["pose_file_date"];
  int pose_counter = 0;

  for (const auto& pose : J["poses"]) {
    pose_counter++;
    ros::Time time_stamp_k;
    time_stamp_k.sec = pose["time_stamp_sec"];
    time_stamp_k.nsec = pose["time_stamp_nsec"];
    beam::Mat4 T_k;
    int i = 0, j = 0;
    int value_counter = 0;
    for (const auto& value : pose["transform"]) {
      value_counter++;
      T_k(i, j) = value.get<double>();
      if (j == 3) {
        i++;
        j = 0;
      } else {
        j++;
      }
    }
    if (value_counter != 16) {
      BEAM_CRITICAL("Invalid transform matrix in json pose file.");
      throw std::invalid_argument{"Invalid transform matrix in .json file."};
    } else {
      time_stamps_.push_back(time_stamp_k);
      Eigen::Affine3d TA_k;
      TA_k.matrix() = T_k;
      poses_.push_back(TA_k);
    }
  }
  BEAM_INFO("Read {} poses.", pose_counter);
}

} // namespace beam_mapping
