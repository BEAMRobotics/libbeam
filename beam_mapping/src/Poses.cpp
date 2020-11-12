#include "beam_mapping/Poses.h"

#include "beam_utils/log.hpp"
#include "beam_utils/math.hpp"

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <nlohmann/json.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2_eigen/tf2_eigen.h>

namespace beam_mapping {

void Poses::Clear() {
  time_stamps.clear();
  poses.clear();
  bag_name = "";
  pose_file_date = "";
  fixed_frame = "";
  moving_frame = "";
}

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

void Poses::WriteToJSON(const std::string output_dir) {
  if (poses.size() != time_stamps.size()) {
    BEAM_CRITICAL("Number of time stamps not equal to number of poses. Not "
                  "outputting to pose file.");
    throw std::runtime_error{"Number of time stamps not equal to number of "
                             "poses. Cannot create pose file."};
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
  for (size_t k = 0; k < poses.size(); k++) {
    beam::Mat4 Tk = poses[k].matrix();
    J_pose_k = {{"time_stamp_sec", time_stamps[k].sec},
                {"time_stamp_nsec", time_stamps[k].nsec},
                {"transform",
                 {Tk(0, 0), Tk(0, 1), Tk(0, 2), Tk(0, 3), Tk(1, 0), Tk(1, 1),
                  Tk(1, 2), Tk(1, 3), Tk(2, 0), Tk(2, 1), Tk(2, 2), Tk(2, 3),
                  Tk(3, 0), Tk(3, 1), Tk(3, 2), Tk(3, 3)}}};
    J_pose_k_string = J_pose_k.dump();
    J_poses_string.erase(J_poses_string.end() - 2,
                         J_poses_string.end()); // erase end ]}
    if (k > 0) { J_poses_string = J_poses_string + ", "; }
    J_poses_string = J_poses_string + J_pose_k_string + "]}"; // add new pose
  }

  J_string.erase(J_string.end() - 1, J_string.end()); // erase end }
  J_poses_string.erase(0, 1);                         // erase start {
  J_string = J_string + "," + J_poses_string;         // add poses
  J = nlohmann::json::parse(J_string);

  std::string output_file;
  std::string last_five_chars =
      output_dir.substr(output_dir.size() - 5, output_dir.size());
  std::cout << "output_dir: " << output_dir << "\n";
  std::cout << "last_five_chars: " << last_five_chars << "\n";
  if (last_five_chars == ".json" || last_five_chars == ".JSON") {
    output_file = output_dir;
    std::cout << "TEST1\n";

  } else {
    if (!boost::filesystem::is_directory(output_dir)) {
      BEAM_INFO("Output directory does not exist, creating now.");
      boost::filesystem::create_directories(output_dir);
    }
    output_file = output_dir + pose_file_date + "_poses.json";
    std::cout << "TEST2\n";
  }
  BEAM_INFO("Saving poses to file: {}", output_file.c_str());
  std::ofstream filejson(output_file);
  filejson << std::setw(4) << J << std::endl;
}

void Poses::LoadFromJSON(const std::string input_pose_file_path) {
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

void Poses::WriteToTXT(const std::string output_dir) {
  if (poses.size() != time_stamps.size()) {
    BEAM_CRITICAL("Number of time stamps not equal to number of poses. Not "
                  "outputting to pose file.");
    throw std::runtime_error{"Number of time stamps not equal to number of "
                             "poses. Cannot create pose file."};
  }
  std::string output_file;
  std::string last_five_chars =
      output_dir.substr(output_dir.size() - 4, output_dir.size());
  if (last_five_chars == ".txt" || last_five_chars == ".TXT") {
    output_file = output_dir;
  } else {
    if (!boost::filesystem::is_directory(output_dir)) {
      BEAM_INFO("Output directory does not exist, creating now.");
      boost::filesystem::create_directories(output_dir);
    }
    output_file = output_dir + pose_file_date + "_poses.txt";
  }

  std::ofstream outfile(output_file);
  for (size_t k = 0; k < poses.size(); k++) {
    beam::Mat4 Tk = poses[k].matrix();
    std::stringstream line;
    line << time_stamps[k].sec;
    // get num digits in nsec
    int length = 1;
    int x = time_stamps[k].nsec;
    while (x /= 10) length++;
    // extend nsec with 0's to fill 9 digits
    if (length < 9) {
      int extend = 9 - length;
      for (int i = 0; i < extend; i++) { line << "0"; }
    }
    line << time_stamps[k].nsec << ", " << Tk(0, 0) << ", " << Tk(0, 1) << ", "
         << Tk(0, 2) << ", " << Tk(0, 3) << ", " << Tk(1, 0) << ", " << Tk(1, 1)
         << ", " << Tk(1, 2) << ", " << Tk(1, 3) << ", " << Tk(2, 0) << ", "
         << Tk(2, 1) << ", " << Tk(2, 2) << ", " << Tk(2, 3) << ", " << Tk(3, 0)
         << ", " << Tk(3, 1) << ", " << Tk(3, 2) << ", " << Tk(3, 3)
         << std::endl;
    std::string line_str = line.str();
    outfile << line_str;
  }
  BEAM_INFO("Saving poses to file: {}", output_file.c_str());
}

void Poses::LoadFromTXT(const std::string input_pose_file_path) {
  // declare variables
  std::ifstream infile;
  std::string line;
  Eigen::Matrix4d Tk;
  int pose_counter = 0;
  ros::Time time_stamp_k;
  // open file
  infile.open(input_pose_file_path);
  // extract poses
  while (!infile.eof()) {
    // get timestamp k
    std::getline(infile, line, ',');
    if (line.length() > 0) {
      try {
        uint64_t n_sec = std::stod(line.substr(line.length() - 9, line.length()));
        uint64_t sec = std::stod(line.substr(0, line.length() - 9));
        time_stamp_k.sec = sec;
        time_stamp_k.nsec = n_sec;
      } catch (const std::invalid_argument& e) {
        BEAM_CRITICAL("Invalid argument, probably at end of file");
        throw std::invalid_argument{
            "Invalid argument, probably at end of file"};
      }

      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
          if (i == 3 && j == 3) {
            std::getline(infile, line, '\n');
            Tk(i, j) = std::stod(line);
          } else {
            std::getline(infile, line, ',');
            Tk(i, j) = std::stod(line);
          }
        }
      }
      time_stamps.push_back(time_stamp_k);
      Eigen::Affine3d TA_k;
      TA_k.matrix() = Tk;
      pose_counter++;
      poses.push_back(TA_k);
    }
  }
  BEAM_INFO("Read {} poses.", pose_counter);
}

void Poses::WriteToPLY(const std::string output_dir) {
  if (poses.size() != time_stamps.size()) {
    BEAM_CRITICAL("Number of time stamps not equal to number of poses. Not "
                  "outputting to pose file.");
    throw std::runtime_error{"Number of time stamps not equal to number of "
                             "poses. Cannot create pose file."};
  }

  if (!boost::filesystem::is_directory(output_dir)) {
    BEAM_INFO("Output directory does not exist, creating now.");
    boost::filesystem::create_directories(output_dir);
  }

  // write to PLY
  std::string output_file;
  std::string last_four_chars =
      output_dir.substr(output_dir.size() - 4, output_dir.size());
  if (last_four_chars == ".ply" || last_four_chars == ".PLY") {
    output_file = output_dir;
  } else {
    output_file = output_dir + pose_file_date + "_poses.ply";
  }
  BEAM_INFO("Saving poses to file: {}", output_file.c_str());
  std::ofstream fileply(output_file);
  double t_start = time_stamps[0].toSec();

  fileply << "ply" << std::endl;
  fileply << "format ascii 1.0" << std::endl;
  fileply << "comment UTC time at start ";
  fileply << std::fixed << std::setprecision(6) << t_start << std::endl;
  fileply << "comment Local time at start " << pose_file_date << std::endl;
  fileply << "bag file: " << bag_name << std::endl;
  fileply << "fixed frame: " << fixed_frame << std::endl;
  fileply << "moving frame: " << moving_frame << std::endl;
  fileply << "element vertex " << time_stamps.size() << std::endl;
  fileply << "property float x" << std::endl;
  fileply << "property float y" << std::endl;
  fileply << "property float z" << std::endl;
  fileply << "property float roll" << std::endl;
  fileply << "property float pitch" << std::endl;
  fileply << "property float yaw" << std::endl;
  fileply << "property float scalar_confidence_metric" << std::endl;
  fileply << "end_header" << std::endl;

  for (size_t k = 0; k < poses.size(); k++) {
    Eigen::RowVector3d Tk = poses[k].translation();
    Eigen::RowVector3d Mk = poses[k].rotation().eulerAngles(0, 1, 2);
    double t = time_stamps[k].toSec() - t_start;
    fileply << std::fixed << std::setprecision(6) << Tk[0] << " ";
    fileply << std::fixed << std::setprecision(6) << Tk[1] << " ";
    fileply << std::fixed << std::setprecision(6) << Tk[2] << " ";
    fileply << std::fixed << std::setprecision(6) << Mk[0] << " ";
    fileply << std::fixed << std::setprecision(6) << Mk[1] << " ";
    fileply << std::fixed << std::setprecision(6) << Mk[2] << " ";
    fileply << std::fixed << std::setprecision(6) << t << " ";
    fileply << std::fixed << std::setprecision(6) << 1.000000 << std::endl;
  }
}

void Poses::LoadFromPLY(const std::string input_pose_file_path) {
  std::string delim = " ";
  std::ifstream file(input_pose_file_path);
  std::string str;
  double time_start = 0;
  while (std::getline(file, str)) {
    if (str.substr(0, 11) == "comment UTC") {
      str.erase(0, 26);
      time_start = std::stod(str);
    }
    if (str.substr(0, 13) == "comment Local") {
      str.erase(0, 28);
      pose_file_date = str;
    }
    if (str.substr(0, 9) == "bag file:") {
      str.erase(0, 10);
      bag_name = str;
    }
    if (str.substr(0, 12) == "fixed frame:") {
      str.erase(0, 13);
      fixed_frame = str;
    }
    if (str.substr(0, 13) == "moving frame:") {
      str.erase(0, 14);
      moving_frame = str;
    }
    if (str == "end_header") { break; }
  }
  std::string s;
  while (std::getline(file, s)) {
    size_t pos = 0;
    std::vector<double> vals;
    while ((pos = s.find(delim)) != std::string::npos) {
      double val = std::stof(s.substr(0, pos));
      s.erase(0, pos + delim.length());
      vals.push_back(val);
    }
    double x = vals[0], y = vals[1], z = vals[2];
    double roll = vals[3], pitch = vals[4], yaw = vals[5];
    double time_since_start = vals[6];
    double time_stamp_sec = time_since_start + time_start;
    ros::Time t(time_stamp_sec);
    Eigen::Affine3d TA;
    Eigen::RowVector3d transl;
    transl << x, y, z;
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::Quaternion<double> q = rollAngle * pitchAngle * yawAngle;
    TA.linear() = q.matrix();
    TA.translation() = transl;
    this->poses.push_back(TA);
    this->time_stamps.push_back(t);
  }
}

void Poses::LoadFromBAG(const std::string bag_file_path,
                        const std::string odom_topic) {
  bag_name = bag_file_path.substr(bag_file_path.rfind("/") + 1,
                                  bag_file_path.rfind(".bag"));
  // open bag
  rosbag::Bag bag;
  try {
    bag.open(bag_file_path, rosbag::bagmode::Read);
  } catch (rosbag::BagException& ex) {
    BEAM_CRITICAL("Bag exception : {}}", ex.what());
  }
  rosbag::View view(bag, rosbag::TopicQuery(odom_topic), ros::TIME_MIN,
                    ros::TIME_MAX, true);
  int total_messages = view.size();
  int message_counter = 0;
  std::string output_message = "Loading odom messages from bag...";
  for (auto iter = view.begin(); iter != view.end(); iter++) {
    message_counter++;
    beam::OutputPercentComplete(message_counter, total_messages,
                                output_message);
    auto odom_msg = iter->instantiate<nav_msgs::Odometry>();
    if (fixed_frame.size() < 2) { fixed_frame = odom_msg->header.frame_id; }
    if (moving_frame.size() < 2) { moving_frame = odom_msg->child_frame_id; }
    time_stamps.push_back(odom_msg->header.stamp);
    Eigen::Affine3d T_MOVING_FIXED;
    Eigen::fromMsg(odom_msg->pose.pose, T_MOVING_FIXED);
    poses.push_back(T_MOVING_FIXED);
  }
  BEAM_INFO("Done loading poses from Bag. Saved {} total poses.",
            message_counter);
}

} // namespace beam_mapping
