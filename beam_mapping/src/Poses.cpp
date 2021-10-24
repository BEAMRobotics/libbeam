#include <beam_mapping/Poses.h>

#include <algorithm>
#include <string>

#include <boost/filesystem.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nlohmann/json.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2_eigen/tf2_eigen.h>

#include <beam_calibration/TfTree.h>
#include <beam_mapping/Utils.h>
#include <beam_utils/log.h>

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

std::string Poses::GetBagName() const {
  return bag_name;
}

void Poses::SetPoseFileDate(const std::string& _pose_file_date) {
  pose_file_date = _pose_file_date;
}

std::string Poses::GetPoseFileDate() const {
  return pose_file_date;
}

void Poses::SetFixedFrame(const std::string& _fixed_frame) {
  fixed_frame = _fixed_frame;
}

std::string Poses::GetFixedFrame() const {
  return fixed_frame;
}

void Poses::SetMovingFrame(const std::string& _moving_frame) {
  moving_frame = _moving_frame;
}

std::string Poses::GetMovingFrame() const {
  return moving_frame;
}

void Poses::SetTimeStamps(const std::vector<ros::Time>& _time_stamps) {
  time_stamps = _time_stamps;
}

std::vector<ros::Time> Poses::GetTimeStamps() const {
  return time_stamps;
}

void Poses::AddSingleTimeStamp(const ros::Time& _time_stamp) {
  time_stamps.push_back(_time_stamp);
}

void Poses::SetPoses(
    const std::vector<Eigen::Matrix4d, beam::AlignMat4d>& _poses) {
  poses = _poses;
}

std::vector<Eigen::Matrix4d, beam::AlignMat4d> Poses::GetPoses() const {
  return poses;
}

void Poses::AddSinglePose(const Eigen::Matrix4d& T_FIXED_MOVING) {
  poses.push_back(T_FIXED_MOVING);
}

void Poses::WriteToJSON(const std::string& output_dir) const {
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
    Eigen::Matrix4d Tk = poses[k];
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

  std::ofstream filejson = CreateFile(output_dir, ".json");
  filejson << std::setw(4) << J << std::endl;
}

void Poses::LoadFromJSON(const std::string& input_pose_file_path) {
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
    Eigen::Matrix4d T_k;
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
      poses.push_back(T_k);
    }
  }
  BEAM_INFO("Read {} poses.", pose_counter);
}

void Poses::WriteToTXT(const std::string& output_dir) const {
  if (poses.size() != time_stamps.size()) {
    BEAM_CRITICAL("Number of time stamps not equal to number of poses. Not "
                  "outputting to pose file.");
    throw std::runtime_error{"Number of time stamps not equal to number of "
                             "poses. Cannot create pose file."};
  }

  std::ofstream outfile = CreateFile(output_dir, ".txt");

  for (size_t k = 0; k < poses.size(); k++) {
    Eigen::Matrix4d Tk = poses[k];
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
}

void Poses::LoadFromTXT(const std::string& input_pose_file_path) {
  // declare variables
  std::ifstream infile;
  std::string line;
  Eigen::Matrix4d Tk;
  ros::Time time_stamp_k;
  // open file
  infile.open(input_pose_file_path);
  // extract poses
  while (!infile.eof()) {
    // get timestamp k
    std::getline(infile, line, ',');
    if (line.length() > 0) {
      try {
        uint64_t n_sec =
            std::stod(line.substr(line.length() - 9, line.length()));
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
      poses.push_back(Tk);
    }
  }
  BEAM_INFO("Read {} poses.", poses.size());
}

void Poses::WriteToPLY(const std::string& output_dir) const {
  if (poses.size() != time_stamps.size()) {
    BEAM_CRITICAL("Number of time stamps not equal to number of poses. Not "
                  "outputting to pose file.");
    throw std::runtime_error{"Number of time stamps not equal to number of "
                             "poses. Cannot create pose file."};
  }

  std::ofstream fileply = CreateFile(output_dir, ".ply");
  double t_start = time_stamps.at(0).toSec();

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
    Eigen::Affine3d TA(poses.at(k));
    Eigen::RowVector3d Tk = TA.translation();
    Eigen::RowVector3d Mk = TA.rotation().eulerAngles(0, 1, 2);
    double t = time_stamps.at(k).toSec() - t_start;
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

void Poses::LoadFromPLY(const std::string& input_pose_file_path) {
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

    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::Quaternion<double> q = rollAngle * pitchAngle * yawAngle;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block(0, 0, 3, 3) = q.matrix();
    T(0, 3) = x;
    T(1, 3) = y;
    T(2, 3) = z;

    poses.push_back(T);
    time_stamps.push_back(t);
  }
}

void Poses::WriteToPLY2(const std::string& output_dir) const {
  if (poses.size() != time_stamps.size()) {
    BEAM_CRITICAL("Number of time stamps not equal to number of poses. Not "
                  "outputting to pose file.");
    throw std::runtime_error{"Number of time stamps not equal to number of "
                             "poses. Cannot create pose file."};
  }

  std::ofstream fileply = CreateFile(output_dir, ".ply");

  const ros::Time& t_start = time_stamps.at(0);

  fileply << "ply" << std::endl;
  fileply << "format ascii 1.0" << std::endl;
  fileply << "comment UTC time at start ";
  fileply << std::fixed << std::setprecision(17) << t_start.toNSec()
          << std::endl;
  fileply << "comment Local time at start " << pose_file_date << std::endl;
  fileply << "comment bag_file: " << bag_name << std::endl;
  fileply << "comment fixed_frame: " << fixed_frame << std::endl;
  fileply << "comment moving_frame: " << moving_frame << std::endl;
  fileply << "element vertex " << time_stamps.size() << std::endl;
  fileply << "property float x" << std::endl;
  fileply << "property float y" << std::endl;
  fileply << "property float z" << std::endl;
  fileply << "property float qw" << std::endl;
  fileply << "property float qx" << std::endl;
  fileply << "property float qy" << std::endl;
  fileply << "property float qz" << std::endl;
  fileply << "property float time_nsec" << std::endl;
  fileply << "end_header" << std::endl;

  for (size_t k = 0; k < poses.size(); k++) {
    const Eigen::Matrix4d& T = poses.at(k);
    Eigen::Matrix3d R = T.block(0, 0, 3, 3);
    Eigen::Quaterniond q(R);
    double t = (time_stamps.at(k) - t_start).toNSec();
    fileply << std::fixed << std::setprecision(7) << T(0, 3) << " ";
    fileply << std::fixed << std::setprecision(7) << T(1, 3) << " ";
    fileply << std::fixed << std::setprecision(7) << T(2, 3) << " ";
    fileply << std::fixed << std::setprecision(7) << q.w() << " ";
    fileply << std::fixed << std::setprecision(7) << q.x() << " ";
    fileply << std::fixed << std::setprecision(7) << q.y() << " ";
    fileply << std::fixed << std::setprecision(7) << q.z() << " ";
    fileply << std::fixed << std::setprecision(16) << t << " " << std::endl;
  }
}

void Poses::LoadFromPLY2(const std::string& input_pose_file_path) {
  std::string delim = " ";
  std::ifstream file(input_pose_file_path);
  std::string str;
  double time_start_double;
  while (std::getline(file, str)) {
    if (str.substr(0, 11) == "comment UTC") {
      str.erase(0, 26);
      time_start_double = std::stod(str);
    }
    if (str.substr(0, 13) == "comment Local") {
      str.erase(0, 28);
      pose_file_date = str;
    }
    if (str.substr(0, 9) == "comment bag_file:") {
      str.erase(0, 10);
      bag_name = str;
    }
    if (str.substr(0, 12) == "comment fixed_frame:") {
      str.erase(0, 13);
      fixed_frame = str;
    }
    if (str.substr(0, 13) == "comment moving_frame:") {
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

    double time_since_start_double = vals[6];
    ros::Time time_stamp;
    time_stamp.fromNSec(time_since_start_double + time_start_double);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 3) = vals[0];
    T(1, 3) = vals[1];
    T(2, 3) = vals[2];
    Eigen::Quaterniond q(vals[3], vals[4], vals[5], vals[6]);
    Eigen::Matrix3d R(q);
    T.block(0, 0, 3, 3) = R;
    poses.push_back(T);

    ros::Time t;
    t.fromNSec(vals[7]);
    time_stamps.push_back(t);
  }
}

void Poses::LoadLoopClosedPaths(const std::string& bag_file_path,
                                const std::string& topic_loop_closed,
                                const std::string& topic_high_rate) {
  boost::filesystem::path p(bag_file_path);
  bag_name = p.stem().string();

  // open bag
  rosbag::Bag bag;
  BEAM_INFO("Opening bag: {}", bag_file_path);
  bag.open(bag_file_path, rosbag::bagmode::Read);

  // load loop closed poses
  rosbag::View view_loop_closed(bag, rosbag::TopicQuery(topic_loop_closed),
                                ros::TIME_MIN, ros::TIME_MAX, true);

  // first, check if input topic is path
  BEAM_INFO("Loading loop closed path messages from topic {}",
            topic_loop_closed);
  for (auto iter = view_loop_closed.begin(); iter != view_loop_closed.end();
       iter++) {
    auto path_msg_test = iter->instantiate<nav_msgs::Path>();
    if (path_msg_test == NULL) {
      BEAM_CRITICAL(
          "Input trajectory message in bag is not of type nav_msgs::Path");
      throw std::runtime_error{"Invalid message type for input message topic."};
    }
    break;
  }

  // get last path message
  boost::shared_ptr<nav_msgs::Path> path_msg;
  for (auto iter = view_loop_closed.begin(); iter != view_loop_closed.end();
       iter++) {
    path_msg = iter->instantiate<nav_msgs::Path>();
  }
  if (path_msg == NULL) {
    throw std::runtime_error{"Cannot instantiate path msg."};
  }

  // convert last path to poses
  pose_map_type loop_closed_poses;
  utils::PathMsgToPoses(*path_msg, loop_closed_poses, fixed_frame,
                        moving_frame);

  // next, load high rate path
  rosbag::View view_high_rate(bag, rosbag::TopicQuery(topic_high_rate),
                              ros::TIME_MIN, ros::TIME_MAX, true);

  // first, check if input topic is path
  BEAM_INFO("Loading high rate path messages from topic {}", topic_high_rate);
  for (auto iter = view_high_rate.begin(); iter != view_high_rate.end();
       iter++) {
    auto path_msg_test = iter->instantiate<nav_msgs::Path>();
    if (path_msg_test == NULL) {
      BEAM_CRITICAL(
          "Input trajectory message in bag is not of type nav_msgs::Path");
      throw std::runtime_error{"Invalid message type for input message topic."};
    }
    break;
  }

  // get all high rate paths
  pose_map_type high_rate_poses;
  int num_duplicate_poses{0};
  for (auto iter = view_high_rate.begin(); iter != view_high_rate.end();
       iter++) {
    path_msg = iter->instantiate<nav_msgs::Path>();
    if (path_msg == NULL) {
      throw std::runtime_error{"Cannot instantiate path msg."};
    }
    num_duplicate_poses += utils::PathMsgToPoses(*path_msg, high_rate_poses,
                                                 fixed_frame, moving_frame);
  }
  BEAM_INFO("Overrode {} duplicate poses.", num_duplicate_poses);

  if (loop_closed_poses.empty() && high_rate_poses.empty()) {
    BEAM_ERROR("No poses read.");
    return;
  } else if (loop_closed_poses.empty()) {
    BEAM_ERROR("No loop closed poses read, using high rate poses only.");
    utils::PoseMapToTimeAndPoseVecs(high_rate_poses, poses, time_stamps);
  } else if (high_rate_poses.empty()) {
    BEAM_ERROR("No high rate poses read, using loop closed poses only.");
    utils::PoseMapToTimeAndPoseVecs(loop_closed_poses, poses, time_stamps);
  } else {
    BEAM_INFO("Correcting {} high rate poses with {} loop closed poses.",
              high_rate_poses.size(), loop_closed_poses.size());
    // convert high rate poses to corrected frame
    auto iter_LC = loop_closed_poses.begin();
    uint64_t t_LC = iter_LC->first;
    Eigen::Matrix4d T_WORLD_BASELINKLC = iter_LC->second;
    Eigen::Matrix4d T_WORLDCORR_WORLDEST = Eigen::Matrix4d::Identity();
    for (auto iter_HR = high_rate_poses.begin();
         iter_HR != high_rate_poses.end(); iter_HR++) {
      const uint64_t& t_HR = iter_HR->first;
      const Eigen::Matrix4d& T_WORLDEST_BASELINKHR = iter_HR->second;

      // if time is equal to or above next LC, then update correction
      if (t_HR >= t_LC) {
        iter_LC++;

        // make sure we're not already at the last LC pose,
        if (iter_LC != loop_closed_poses.end()) {
          // BEAM_INFO("Updateing correction");
          t_LC = iter_LC->first;
          T_WORLD_BASELINKLC = iter_LC->second;
          T_WORLDCORR_WORLDEST =
              T_WORLD_BASELINKLC * beam::InvertTransform(T_WORLDEST_BASELINKHR);
        }
      }

      // correct pose and add
      // BEAM_INFO("Adding corrected pose.");
      Eigen::Matrix4d T_WORLDCORR_BASELINKHR =
          T_WORLDCORR_WORLDEST * T_WORLDEST_BASELINKHR;
      ros::Time new_stamp;
      new_stamp.fromNSec(t_HR);
      time_stamps.push_back(new_stamp);
      poses.push_back(T_WORLDCORR_BASELINKHR);
    }
  }

  // check frames have been set, if not set defaults
  if (fixed_frame.empty()) { fixed_frame = "odom"; }
  if (moving_frame.empty()) { moving_frame = "base_link"; }
  BEAM_INFO("Done loading poses from Bag. Saved {} total poses.", poses.size());
}

void Poses::LoadLoopClosedPathsInterpolated(
    const std::string& bag_file_path, const std::string& topic_loop_closed,
    const std::string& topic_high_rate) {
  boost::filesystem::path p(bag_file_path);
  bag_name = p.stem().string();

  // open bag
  rosbag::Bag bag;
  BEAM_INFO("Opening bag: {}", bag_file_path);
  bag.open(bag_file_path, rosbag::bagmode::Read);

  // load loop closed poses
  rosbag::View view_loop_closed(bag, rosbag::TopicQuery(topic_loop_closed),
                                ros::TIME_MIN, ros::TIME_MAX, true);

  // first, check if input topic is path
  BEAM_INFO("Loading loop closed path messages from topic {}",
            topic_loop_closed);
  for (auto iter = view_loop_closed.begin(); iter != view_loop_closed.end();
       iter++) {
    auto path_msg_test = iter->instantiate<nav_msgs::Path>();
    if (path_msg_test == NULL) {
      BEAM_CRITICAL(
          "Input trajectory message in bag is not of type nav_msgs::Path");
      throw std::runtime_error{"Invalid message type for input message topic."};
    }
    break;
  }

  // get last path message
  boost::shared_ptr<nav_msgs::Path> path_msg;
  for (auto iter = view_loop_closed.begin(); iter != view_loop_closed.end();
       iter++) {
    path_msg = iter->instantiate<nav_msgs::Path>();
  }
  if (path_msg == NULL) {
    throw std::runtime_error{"Cannot instantiate path msg."};
  }

  // convert last path to tf tree
  pose_map_type loop_closed_poses;
  utils::PathMsgToPoses(*path_msg, loop_closed_poses, fixed_frame,
                        moving_frame);

  // next, load high rate path
  rosbag::View view_high_rate(bag, rosbag::TopicQuery(topic_high_rate),
                              ros::TIME_MIN, ros::TIME_MAX, true);

  // first, check if input topic is path
  BEAM_INFO("Loading high rate path messages from topic {}", topic_high_rate);
  for (auto iter = view_high_rate.begin(); iter != view_high_rate.end();
       iter++) {
    auto path_msg_test = iter->instantiate<nav_msgs::Path>();
    if (path_msg_test == NULL) {
      BEAM_CRITICAL(
          "Input trajectory message in bag is not of type nav_msgs::Path");
      throw std::runtime_error{"Invalid message type for input message topic."};
    }
    break;
  }

  // get all high rate paths
  pose_map_type high_rate_poses;
  int num_duplicate_poses{0};
  for (auto iter = view_high_rate.begin(); iter != view_high_rate.end();
       iter++) {
    path_msg = iter->instantiate<nav_msgs::Path>();
    if (path_msg == NULL) {
      throw std::runtime_error{"Cannot instantiate path msg."};
    }
    num_duplicate_poses += utils::PathMsgToPoses(*path_msg, high_rate_poses,
                                                 fixed_frame, moving_frame);
  }
  BEAM_INFO("Overrode {} duplicate poses.", num_duplicate_poses);

  if (loop_closed_poses.empty() && high_rate_poses.empty()) {
    BEAM_ERROR("No poses read.");
    return;
  } else if (loop_closed_poses.empty()) {
    BEAM_ERROR("No loop closed poses read, using high rate poses only.");
    utils::PoseMapToTimeAndPoseVecs(high_rate_poses, poses, time_stamps);
    // check frames have been set, if not set defaults
    if (fixed_frame.empty()) { fixed_frame = "odom"; }
    if (moving_frame.empty()) { moving_frame = "base_link"; }
    BEAM_INFO("Done loading poses from Bag. Saved {} total poses.",
              poses.size());
    return;
  } else if (high_rate_poses.empty()) {
    BEAM_ERROR("No high rate poses read, using loop closed poses only.");
    utils::PoseMapToTimeAndPoseVecs(loop_closed_poses, poses, time_stamps);
    // check frames have been set, if not set defaults
    if (fixed_frame.empty()) { fixed_frame = "odom"; }
    if (moving_frame.empty()) { moving_frame = "base_link"; }
    BEAM_INFO("Done loading poses from Bag. Saved {} total poses.",
              poses.size());
    return;
  }

  // correct poses using a tf tree of corrections
  BEAM_INFO("Correcting {} high rate poses with {} loop closed poses.",
            high_rate_poses.size(), loop_closed_poses.size());
  beam_calibration::TfTree corrections;

  // if first HR pose comes before first LC pose, then add identity for first
  // correction
  if (high_rate_poses.begin()->first < loop_closed_poses.begin()->first) {
    Eigen::Affine3d T(Eigen::Matrix4d::Identity());
    ros::Time stamp;
    stamp.fromNSec(high_rate_poses.begin()->first);
    corrections.AddTransform(T, "WORLD_CORRECTED", "WORLD_ESTIMATED", stamp);
  }

  // iterate through HR poses and build corrections
  auto tmp_LC_iter = loop_closed_poses.begin();
  Eigen::Matrix4d T_WORLDCORR_WORLDEST;
  for (auto iter_HR = high_rate_poses.begin(); iter_HR != high_rate_poses.end();
       iter_HR++) {
    const uint64_t& t_HR = iter_HR->first;

    // if time of current HR pose is greater or equal to LC pose, then add
    // correction and increment LC iter
    if (t_HR >= tmp_LC_iter->first && tmp_LC_iter != loop_closed_poses.end()) {
      // get correction
      const Eigen::Matrix4d& T_WORLDEST_BASELINKHR = iter_HR->second;
      const Eigen::Matrix4d& T_WORLD_BASELINKLC = tmp_LC_iter->second;
      tmp_LC_iter++;
      T_WORLDCORR_WORLDEST =
          T_WORLD_BASELINKLC * beam::InvertTransform(T_WORLDEST_BASELINKHR);

      // add correction to tf tree
      Eigen::Affine3d T(T_WORLDCORR_WORLDEST);
      ros::Time stamp;
      stamp.fromNSec(t_HR);
      corrections.AddTransform(T, "WORLD_CORRECTED", "WORLD_ESTIMATED", stamp);
    }
  }

  // if last HR pose is after last LC pose, then add a correction equal to the
  // final correction
  if (loop_closed_poses.rbegin()->first < high_rate_poses.rbegin()->first) {
    Eigen::Affine3d T(T_WORLDCORR_WORLDEST);
    ros::Time stamp;
    stamp.fromNSec(high_rate_poses.rbegin()->first);
    corrections.AddTransform(T, "WORLD_CORRECTED", "WORLD_ESTIMATED", stamp);
  }

  // Correct all HR poses by interpolating corrections
  for (auto iter_HR = high_rate_poses.begin(); iter_HR != high_rate_poses.end();
       iter_HR++) {
    const uint64_t& t_HR = iter_HR->first;
    const Eigen::Matrix4d& T_WORLDEST_BASELINKHR = iter_HR->second;

    // calculate correction
    ros::Time stamp_HR;
    stamp_HR.fromNSec(t_HR);
    T_WORLDCORR_WORLDEST =
        corrections
            .GetTransformEigen("WORLD_CORRECTED", "WORLD_ESTIMATED", stamp_HR)
            .matrix();

    // correct pose and add
    Eigen::Matrix4d T_WORLDCORR_BASELINKHR =
        T_WORLDCORR_WORLDEST * T_WORLDEST_BASELINKHR;
    time_stamps.push_back(stamp_HR);
    poses.push_back(T_WORLDCORR_BASELINKHR);
  }
}

void Poses::LoadFromBAG(const std::string& bag_file_path,
                        const std::string& topic) {
  boost::filesystem::path p(bag_file_path);
  bag_name = p.stem().string();

  // open bag
  rosbag::Bag bag;
  BEAM_INFO("Opening bag: {}", bag_file_path);
  bag.open(bag_file_path, rosbag::bagmode::Read);

  rosbag::View view(bag, rosbag::TopicQuery(topic), ros::TIME_MIN,
                    ros::TIME_MAX, true);

  // first, check if input topic is odom or path
  bool is_odom{true};
  for (auto iter = view.begin(); iter != view.end(); iter++) {
    auto odom_msg = iter->instantiate<nav_msgs::Odometry>();
    if (odom_msg == NULL) {
      is_odom = false;
    } else {
      break;
    }

    auto path_msg = iter->instantiate<nav_msgs::Path>();
    if (path_msg == NULL) {
      BEAM_CRITICAL("Input trajectory message in bag is not of type "
                    "nav_msgs::Odometry or nav_msgs::Path");
      throw std::runtime_error{"Invalid message type for input message topic."};
    }
    break;
  }

  if (is_odom) {
    BEAM_INFO("Loading odom messages from bag");
    for (auto iter = view.begin(); iter != view.end(); iter++) {
      auto odom_msg = iter->instantiate<nav_msgs::Odometry>();
      if (odom_msg == NULL) {
        throw std::runtime_error{"Cannot instantiate odometry msg."};
      }

      if (fixed_frame.empty()) { fixed_frame = odom_msg->header.frame_id; }
      if (moving_frame.empty()) { moving_frame = odom_msg->child_frame_id; }
      time_stamps.push_back(odom_msg->header.stamp);
      Eigen::Affine3d T_FIXED_MOVING;
      Eigen::fromMsg(odom_msg->pose.pose, T_FIXED_MOVING);
      poses.push_back(T_FIXED_MOVING.matrix());
    }
  } else {
    BEAM_INFO("Loading path messages from bag");
    boost::shared_ptr<nav_msgs::Path> path_msg;
    // get last path message
    for (auto iter = view.begin(); iter != view.end(); iter++) {
      path_msg = iter->instantiate<nav_msgs::Path>();
    }

    if (path_msg == NULL) {
      throw std::runtime_error{"Cannot instantiate path msg."};
    }

    // convert to poses
    utils::PathMsgToPoses(*path_msg, poses, time_stamps, fixed_frame,
                          moving_frame);
  }

  if (fixed_frame.empty()) { fixed_frame = "odom"; }
  if (moving_frame.empty()) { moving_frame = "base_link"; }
  BEAM_INFO("Done loading poses from Bag. Saved {} total poses.", poses.size());
}

void Poses::LoadFromPCD(const std::string& input_pose_file_path) {
  // creat cloud ptr
  pcl::PointCloud<PointXYZIRPYT>::Ptr cloud =
      std::make_shared<pcl::PointCloud<PointXYZIRPYT>>();

  // load PCD
  int pcl_loaded =
      pcl::io::loadPCDFile<PointXYZIRPYT>(input_pose_file_path, *cloud);
  if (pcl_loaded == -1) {
    BEAM_CRITICAL("Cannot load poses from PCD file");
    throw std::runtime_error{
        "Check point type used in PCD. Cannot create pose file."};
  }

  // process PCD
  for (const auto& point : *cloud) {
    Eigen::Matrix4d T;
    ros::Time t;
    beam::PCLPointToPose(point, t, T);
    poses.push_back(T);
    time_stamps.push_back(t);
  }
}

std::ofstream Poses::CreateFile(const std::string& output_path,
                                const std::string& extension) const {
  // get lowercase of extension
  std::string output_ext = extension;
  std::transform(output_ext.begin(), output_ext.end(), output_ext.begin(),
                 ::tolower);

  // check directory exists, if not try to create
  boost::filesystem::path p(output_path);
  boost::filesystem::path dir = p.parent_path();
  if (!boost::filesystem::is_directory(dir)) {
    BEAM_INFO("Output directory does not exist, creating now.");
    if (!boost::filesystem::create_directories(dir)) {
      BEAM_CRITICAL("Cannot create directories {}, from output path {}.",
                    dir.string(), output_path);
      throw std::runtime_error{"Cannot create directories"};
    }
  }

  // get full filename
  std::string output_file;
  std::string ext_read = p.extension().string();
  if (!ext_read.empty()) {
    // if extension provided, check that it's consistent with expected
    std::string ext_read_lowercase = extension;
    std::transform(ext_read_lowercase.begin(), ext_read_lowercase.end(),
                   ext_read_lowercase.begin(), ::tolower);
    if (ext_read_lowercase != output_ext) {
      BEAM_ERROR("Extension in output path not consistent with expected "
                 "extension, using expected extension ({})",
                 output_ext);
    }
    output_file = dir.string() + p.stem().string() + output_ext;
  } else {
    // if no extension provided in output path, then we post-fix
    if (output_path.back() == '/') {
      // ending in '/', therefore post-fix with poses.extension
      output_file = output_path + "poses" + output_ext;
    } else {
      // not ending in '/', therefore post-fix with _poses.extension
      output_file = output_path + "_poses" + output_ext;
    }
  }

  BEAM_INFO("Saving poses to file: {}", output_file);
  return std::ofstream(output_file);
}
} // namespace beam_mapping
