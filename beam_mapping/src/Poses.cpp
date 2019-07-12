#include "beam_mapping/Poses.h"

#include "beam_utils/log.hpp"
#include "beam_utils/math.hpp"

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

namespace beam_mapping {

void Poses::SetBagName(const std::string& _bag_name) {
  bag_name = _bag_name;
}

std::string Poses::GetBagName() {
  return bag_name;
}

void Poses::SetPoseFileDate(const std::string& _pose_file_date) {
  pose_file_date = _pose_file_date;
}

std::string Poses::GetPoseFileDate() {
  return pose_file_date;
}

void Poses::SetFixedFrame(const std::string& _fixed_frame) {
  fixed_frame = _fixed_frame;
}

std::string Poses::GetFixedFrame() {
  return fixed_frame;
}

void Poses::SetMovingFrame(const std::string& _moving_frame) {
  moving_frame = _moving_frame;
}

std::string Poses::GetMovingFrame() {
  return moving_frame;
}

void Poses::SetTimeStamps(const std::vector<ros::Time>& _time_stamps) {
  time_stamps = _time_stamps;
}

std::vector<ros::Time> Poses::GetTimeStamps() {
  return time_stamps;
}

void Poses::AddSingleTimeStamp(const ros::Time& _time_stamp) {
  time_stamps.push_back(_time_stamp);
}

void Poses::SetPoses(
    const std::vector<Eigen::Affine3d,
                      Eigen::aligned_allocator<Eigen::Affine3d>>& _poses) {
  poses = _poses;
}

std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>
    Poses::GetPoses() {
  return poses;
}

void Poses::AddSinglePose(const Eigen::Affine3d& pose) {
  poses.push_back(pose);
}

void Poses::WriteToPoseFile(const std::string output_dir) {
  if (poses.size() != time_stamps.size()) {
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
  J = {{"bag_name", bag_name},
       {"pose_file_date", pose_file_date},
       {"fixed_frame", fixed_frame},
       {"moving_frame", moving_frame}};

  J_string = J.dump();
  J_poses_string = "{\"poses\": []}";
  for (uint64_t k = 0; k < poses.size(); k++) {
    beam::Mat4 Tk = poses[k].matrix();
    J_pose_k = {{"time_stamp_sec", time_stamps[k].sec},
                {"time_stamp_nsec", time_stamps[k].nsec},
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

  std::string output_file = output_dir + pose_file_date + "_poses.json";
  BEAM_INFO("Saving poses to file: {}", output_file.c_str());
  std::ofstream filejson(output_file);
  filejson << std::setw(4) << J << std::endl;
}

void Poses::LoadPoseFile(const std::string input_pose_file_path) {
  BEAM_INFO("Loading pose file: {}", input_pose_file_path.c_str());
  nlohmann::json J;
  std::ifstream file(input_pose_file_path);
  file >> J;
  bag_name = J["bag_name"];
  fixed_frame = J["fixed_frame"];
  moving_frame = J["moving_frame"];
  pose_file_date = J["pose_file_date"];
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
      time_stamps.push_back(time_stamp_k);
      Eigen::Affine3d TA_k;
      TA_k.matrix() = T_k;
      poses.push_back(TA_k);
    }
  }
  BEAM_INFO("Read {} poses.", pose_counter);
}

} // namespace beam_mapping
