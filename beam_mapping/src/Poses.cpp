#include <beam_mapping/Poses.h>

#include <algorithm>
#include <string>

#include <boost/filesystem.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nlohmann/json.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2_eigen/tf2_eigen.h>

#include <beam_calibration/TfTree.h>
#include <beam_mapping/Utils.h>
#include <beam_utils/filesystem.h>
#include <beam_utils/log.h>
#include <beam_utils/math.h>
#include <beam_utils/se3.h>

namespace beam_mapping {

void Poses::Clear() {
  time_stamps_.clear();
  poses_.clear();
  bag_name_ = "";
  pose_file_date_ = "";
  fixed_frame_ = "";
  moving_frame_ = "";
}

void Poses::SetBagName(const std::string& bag_name) {
  bag_name_ = bag_name;
}

std::string Poses::GetBagName() const {
  return bag_name_;
}

void Poses::SetPoseFileDate(const std::string& pose_file_date) {
  pose_file_date_ = pose_file_date;
}

std::string Poses::GetPoseFileDate() const {
  return pose_file_date_;
}

void Poses::SetFixedFrame(const std::string& fixed_frame) {
  fixed_frame_ = fixed_frame;
}

std::string Poses::GetFixedFrame() const {
  return fixed_frame_;
}

void Poses::SetMovingFrame(const std::string& moving_frame) {
  moving_frame_ = moving_frame;
}

std::string Poses::GetMovingFrame() const {
  return moving_frame_;
}

void Poses::SetTimeStamps(const std::vector<ros::Time>& time_stamps) {
  time_stamps_ = time_stamps;
}

const std::vector<ros::Time>& Poses::GetTimeStamps() const {
  return time_stamps_;
}

void Poses::AddSingleTimeStamp(const ros::Time& time_stamp) {
  time_stamps_.push_back(time_stamp);
}

void Poses::SetPoses(
    const std::vector<Eigen::Matrix4d, beam::AlignMat4d>& poses) {
  poses_ = poses;
}

const std::vector<Eigen::Matrix4d, beam::AlignMat4d>& Poses::GetPoses() const {
  return poses_;
}

void Poses::AddSinglePose(const Eigen::Matrix4d& T_FIXED_MOVING) {
  poses_.push_back(T_FIXED_MOVING);
}

bool Poses::LoadFromFile(const std::string& input_pose_file_path,
                         int format_type) {
  std::string file_type = input_pose_file_path.substr(
      input_pose_file_path.rfind("."), input_pose_file_path.size());
  if (file_type == ".json") {
    LoadFromJSON(input_pose_file_path);
  } else if (file_type == ".ply") {
    LoadFromPLY(input_pose_file_path, format_type);
  } else if (file_type == ".txt") {
    LoadFromTXT(input_pose_file_path, format_type);
  } else if (file_type == ".pcd") {
    LoadFromPCD(input_pose_file_path);
  } else {
    return false;
  }
  BEAM_INFO("Read {} poses.", poses_.size());
  return true;
}

bool Poses::WriteToFile(const std::string& output_dir,
                        const std::string& file_type, int format_type) {
  if (file_type == "JSON") {
    WriteToJSON(output_dir);
  } else if (file_type == "PLY") {
    WriteToPLY(output_dir, format_type);
  } else if (file_type == "TXT") {
    WriteToTXT(output_dir, format_type);
  } else if (file_type == "PCD") {
    WriteCoordinateFramesToPCD(output_dir);
  } else {
    BEAM_ERROR("Invalid file type, using default: JSON");
    WriteToJSON(output_dir);
  }
  return true;
}

void Poses::WriteToJSON(const std::string& output_dir) const {
  CheckPoses();

  // write to json
  nlohmann::json J = {{"bag_name", bag_name_},
                      {"pose_file_date", pose_file_date_},
                      {"fixed_frame", fixed_frame_},
                      {"moving_frame", moving_frame_}};

  std::string J_string = J.dump();
  std::string J_poses_string = "{\"poses\": []}";
  for (size_t k = 0; k < poses_.size(); k++) {
    Eigen::Matrix4d Tk = poses_[k];
    nlohmann::json J_pose_k = {
        {"time_stamp_sec", time_stamps_[k].sec},
        {"time_stamp_nsec", time_stamps_[k].nsec},
        {"transform",
         {Tk(0, 0), Tk(0, 1), Tk(0, 2), Tk(0, 3), Tk(1, 0), Tk(1, 1), Tk(1, 2),
          Tk(1, 3), Tk(2, 0), Tk(2, 1), Tk(2, 2), Tk(2, 3), Tk(3, 0), Tk(3, 1),
          Tk(3, 2), Tk(3, 3)}}};
    std::string J_pose_k_string = J_pose_k.dump();
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

void Poses::WriteCoordinateFramesToPCD(const std::string& output_dir) const {
  CheckPoses();

  BEAM_INFO("Converting poses to pointclouds");
  pcl::PointCloud<pcl::PointXYZRGBL> cloud;
  // to help prevent time variable overflow, start first timestamp as 0
  const double t_start = time_stamps_.at(0).toSec();
  for (size_t k = 0; k < poses_.size(); k++) {
    Eigen::Matrix4d Tk = poses_[k];
    ros::Time tk;
    tk.fromSec(time_stamps_[k].toSec() - t_start);
    pcl::PointCloud<pcl::PointXYZRGBL> frame = beam::CreateFrameCol(tk);
    beam::MergeFrameToCloud(cloud, frame, Tk);
  }

  std::string filepcd = GetOutputFileName(output_dir, ".pcd");
  BEAM_INFO("Saving poses cloud file to: {}", filepcd);
  beam::SavePointCloud<pcl::PointXYZRGBL>(filepcd, cloud);
}

void Poses::LoadFromJSON(const std::string& input_pose_file_path) {
  time_stamps_.clear();
  poses_.clear();

  BEAM_INFO("Loading pose file: {}", input_pose_file_path);
  nlohmann::json J;
  if (!beam::ReadJson(input_pose_file_path, J)) {
    throw std::runtime_error{"Invalid json"};
  }

  try {
    bag_name_ = J["bag_name"];
    fixed_frame_ = J["fixed_frame"];
    moving_frame_ = J["moving_frame"];
    pose_file_date_ = J["pose_file_date"];
    for (const auto& pose : J["poses"]) {
      ros::Time time_stamp_k;
      time_stamp_k.sec = pose["time_stamp_sec"];
      time_stamp_k.nsec = pose["time_stamp_nsec"];
      std::vector<double> T_k_vec = pose["transform"];
      Eigen::Matrix4d T_k = beam::VectorToEigenTransform(T_k_vec);
      time_stamps_.push_back(time_stamp_k);
      poses_.push_back(T_k);
    }
  } catch (const nlohmann::json::exception& e) {
    BEAM_CRITICAL("Unable to load json, one or more missing or invalid params. "
                  "Reason: {}",
                  e.what());
    throw std::runtime_error{"Invalid json"};
  }
}

void Poses::WriteToTXT(const std::string& output_dir, int format_type) const {
  CheckPoses();
  if (format_type != format_type::Type1 && format_type != format_type::Type2) {
    BEAM_ERROR("Invalid format_type, using default: Type1");
    format_type = format_type::Type1;
  }

  std::ofstream outfile = CreateFile(output_dir, ".txt");

  switch (format_type) {
    case format_type::Type1: {
      for (size_t k = 0; k < poses_.size(); k++) {
        const Eigen::Matrix4d& T = poses_.at(k);
        outfile << std::fixed << std::setprecision(9)
                << time_stamps_[k].toNSec() << ", ";
        outfile << std::fixed << std::setprecision(9) << T(0, 0) << ", ";
        outfile << std::fixed << std::setprecision(9) << T(0, 1) << ", ";
        outfile << std::fixed << std::setprecision(9) << T(0, 2) << ", ";
        outfile << std::fixed << std::setprecision(9) << T(0, 3) << ", ";
        outfile << std::fixed << std::setprecision(9) << T(1, 0) << ", ";
        outfile << std::fixed << std::setprecision(9) << T(1, 1) << ", ";
        outfile << std::fixed << std::setprecision(9) << T(1, 2) << ", ";
        outfile << std::fixed << std::setprecision(9) << T(1, 3) << ", ";
        outfile << std::fixed << std::setprecision(9) << T(2, 0) << ", ";
        outfile << std::fixed << std::setprecision(9) << T(2, 1) << ", ";
        outfile << std::fixed << std::setprecision(9) << T(2, 2) << ", ";
        outfile << std::fixed << std::setprecision(9) << T(2, 3) << ", ";
        outfile << std::fixed << std::setprecision(9) << T(3, 0) << ", ";
        outfile << std::fixed << std::setprecision(9) << T(3, 1) << ", ";
        outfile << std::fixed << std::setprecision(9) << T(3, 2) << ", ";
        outfile << std::fixed << std::setprecision(9) << T(3, 3) << std::endl;
      }
      break;
    }
    case format_type::Type2: {
      outfile << "# time x y z qx qy qz qw" << std::endl;
      for (size_t k = 0; k < poses_.size(); k++) {
        const Eigen::Matrix4d& T = poses_.at(k);
        Eigen::Vector3d p;
        Eigen::Quaterniond q;
        beam::TransformMatrixToQuaternionAndTranslation(T, q, p);

        outfile << std::fixed << std::setprecision(9) << time_stamps_[k].toSec()
                << " ";
        outfile << std::fixed << std::setprecision(9) << p[0] << " ";
        outfile << std::fixed << std::setprecision(9) << p[1] << " ";
        outfile << std::fixed << std::setprecision(9) << p[2] << " ";
        outfile << std::fixed << std::setprecision(9) << q.x() << " ";
        outfile << std::fixed << std::setprecision(9) << q.y() << " ";
        outfile << std::fixed << std::setprecision(9) << q.z() << " ";
        outfile << std::fixed << std::setprecision(9) << q.w() << std::endl;
      }
      break;
    }
  }
}

void Poses::LoadFromTXT(const std::string& input_pose_file_path,
                        int format_type) {
  time_stamps_.clear();
  poses_.clear();

  if (format_type != format_type::Type1 && format_type != format_type::Type2) {
    BEAM_ERROR("Invalid format_type, using default: Type1");
    format_type = format_type::Type1;
  }

  std::ifstream file(input_pose_file_path);
  std::string s;
  std::string delim{","};
  if (format_type == format_type::Type2) { delim = " "; };

  while (std::getline(file, s)) {
    if (s[0] == '#') { continue; }

    std::vector<double> vals;
    if (beam::StringToNumericValues(delim, s, vals)) {
      ros::Time time_stamp;
      Eigen::Matrix4d T;

      switch (format_type) {
        case format_type::Type1: {
          std::string time_string = std::to_string(vals[0]);

          // remove trailing zeros from double to string conversion
          std::size_t loc = time_string.find(".");
          time_string = time_string.substr(0, loc);

          // nsec and sec fields are fixed
          uint64_t n_sec = std::stod(time_string.substr(
              time_string.length() - 9, time_string.length()));
          uint64_t sec =
              std::stod(time_string.substr(0, time_string.length() - 9));
          time_stamp.sec = sec;
          time_stamp.nsec = n_sec;

          // reshape transformation matrix from vector to matrix
          std::vector<double> pose(vals.begin() + 1, vals.end());
          T = beam::VectorToEigenTransform(pose);
          break;
        }
        case format_type::Type2: {
          ros::Time time_stamp_temp(vals[0]);
          time_stamp = time_stamp_temp;
          Eigen::Vector3d p(vals[1], vals[2], vals[3]);
          Eigen::Quaterniond q(vals[7], vals[4], vals[5], vals[6]);
          beam::QuaternionAndTranslationToTransformMatrix(q, p, T);
          break;
        }
      }
      time_stamps_.push_back(time_stamp);
      poses_.push_back(T);
    }
  }
}

void Poses::WriteToPLY(const std::string& output_dir, int format_type) const {
  if (poses_.size() != time_stamps_.size()) {
    BEAM_CRITICAL("Number of time stamps not equal to number of poses. Not "
                  "outputting to pose file.");
    throw std::runtime_error{"Number of time stamps not equal to number of "
                             "poses. Cannot create pose file."};
  }

  std::ofstream outfile = CreateFile(output_dir, ".ply");
  const ros::Time& t_start = time_stamps_.at(0);

  outfile << "ply" << std::endl;
  outfile << "format ascii 1.0" << std::endl;
  outfile << "comment UTC time at start ";
  outfile << std::fixed << std::setprecision(6) << t_start.toSec() << std::endl;
  outfile << "comment Local time at start " << pose_file_date_ << std::endl;
  outfile << "comment bag_file " << bag_name_ << std::endl;
  outfile << "comment fixed_frame " << fixed_frame_ << std::endl;
  outfile << "comment moving_frame " << moving_frame_ << std::endl;

  switch (format_type) {
    case format_type::Type1: {
      outfile << "comment orientation_type RPY" << std::endl;
      outfile << "element vertex " << time_stamps_.size() << std::endl;
      outfile << "property float x" << std::endl;
      outfile << "property float y" << std::endl;
      outfile << "property float z" << std::endl;
      outfile << "property float roll" << std::endl;
      outfile << "property float pitch" << std::endl;
      outfile << "property float yaw" << std::endl;
      outfile << "property float time" << std::endl;
      outfile << "property float scalar_confidence_metric" << std::endl;
      outfile << "end_header" << std::endl;

      for (size_t k = 0; k < poses_.size(); k++) {
        double t = (time_stamps_.at(k) - t_start).toSec();
        const Eigen::Matrix4d& T = poses_.at(k);
        Eigen::Vector3d p;
        Eigen::Vector3d euler;
        Eigen::Quaterniond q;
        beam::TransformMatrixToQuaternionAndTranslation(T, q, p);
        beam::QuaterniontoRPY(q, euler);
        outfile << std::fixed << std::setprecision(9) << p.x() << " ";
        outfile << std::fixed << std::setprecision(9) << p.y() << " ";
        outfile << std::fixed << std::setprecision(9) << p.z() << " ";
        outfile << std::fixed << std::setprecision(9) << euler.x() << " ";
        outfile << std::fixed << std::setprecision(9) << euler.y() << " ";
        outfile << std::fixed << std::setprecision(9) << euler.z() << " ";
        outfile << std::fixed << std::setprecision(9) << t << " ";
        outfile << std::fixed << std::setprecision(9) << 1.000000 << std::endl;
      }
      break;
    }
    case format_type::Type2: {
      outfile << "comment orientation_type Quaternion" << std::endl;
      outfile << "element vertex " << time_stamps_.size() << std::endl;
      outfile << "property float x" << std::endl;
      outfile << "property float y" << std::endl;
      outfile << "property float z" << std::endl;
      outfile << "property float qw" << std::endl;
      outfile << "property float qx" << std::endl;
      outfile << "property float qy" << std::endl;
      outfile << "property float qz" << std::endl;
      outfile << "property float time_nsec" << std::endl;
      outfile << "end_header" << std::endl;

      for (size_t k = 0; k < poses_.size(); k++) {
        double t = (time_stamps_.at(k) - t_start).toNSec();
        const Eigen::Matrix4d& T = poses_.at(k);
        Eigen::Vector3d p;
        Eigen::Quaterniond q;
        beam::TransformMatrixToQuaternionAndTranslation(T, q, p);
        outfile << std::fixed << std::setprecision(9) << p.x() << " ";
        outfile << std::fixed << std::setprecision(9) << p.y() << " ";
        outfile << std::fixed << std::setprecision(9) << p.z() << " ";
        outfile << std::fixed << std::setprecision(9) << q.w() << " ";
        outfile << std::fixed << std::setprecision(9) << q.x() << " ";
        outfile << std::fixed << std::setprecision(9) << q.y() << " ";
        outfile << std::fixed << std::setprecision(9) << q.z() << " ";
        outfile << std::fixed << std::setprecision(0) << t << std::endl;
      }
      break;
    }
  }
}

void Poses::LoadFromPLY(const std::string& input_pose_file_path,
                        int format_type) {
  time_stamps_.clear();
  poses_.clear();

  std::ifstream file(input_pose_file_path);
  double start_time_sec{0};
  double start_time_nsec{0};
  std::string str;
  std::string orientation_type;
  std::string string1 = "comment UTC time at start ";
  std::string string2 = "comment Local time at start ";
  std::string string3 = "comment bag_file ";
  std::string string4 = "comment fixed_frame ";
  std::string string5 = "comment moving_frame ";
  std::string string6 = "comment orientation_type ";

  while (std::getline(file, str)) {
    if (str.substr(0, string1.size()) == string1) {
      str.erase(0, string1.size());
      size_t pos = str.find(".");
      start_time_sec = std::stod(str.substr(0, pos));
      str.erase(0, pos + 1);
      while (str.size() < 9) { str += "0"; }
      start_time_nsec = std::stod(str);
    }
    if (str.substr(0, string2.size()) == string2) {
      str.erase(0, string2.size());
      pose_file_date_ = str;
    }
    if (str.substr(0, string3.size()) == string3) {
      str.erase(0, string3.size());
      bag_name_ = str;
    }
    if (str.substr(0, string4.size()) == string4) {
      str.erase(0, string4.size());
      fixed_frame_ = str;
    }
    if (str.substr(0, string5.size()) == string5) {
      str.erase(0, string5.size());
      moving_frame_ = str;
    }
    if (str.substr(0, string6.size()) == string6) {
      str.erase(0, string6.size());
      orientation_type = str;
    }
    if (str == "end_header") { break; }
  }

  if (!orientation_type.empty()) {
    if (boost::iequals(orientation_type, "RPY"))
      format_type = format_type::Type1;
    else if (boost::iequals(orientation_type, "Quaternion")) {
      format_type = format_type::Type2;
    } else {
      BEAM_ERROR("Invalid or missing orientation_type in ply header. Assuming "
                 "type is RPY");
      format_type = format_type::Type1;
    }
  } else if (format_type != format_type::Type1 &&
             format_type != format_type::Type2) {
    BEAM_ERROR("Invalid format_type. Assuming Type1");
    format_type = format_type::Type1;
  }

  ros::Time time_start(start_time_sec, start_time_nsec);
  std::string delim = " ";
  std::string s;

  while (std::getline(file, s)) {
    std::vector<double> vals;
    if (beam::StringToNumericValues(delim, s, vals)) {
      ros::Duration duration;
      Eigen::Quaterniond q;

      switch (format_type) {
        case format_type::Type1: {
          duration.fromSec(vals[6]);
          beam::RPYtoQuaternion(vals[3], vals[4], vals[5], q);
          break;
        }
        case format_type::Type2: {
          duration.fromNSec(vals[7]);
          q.w() = vals[3];
          q.x() = vals[4];
          q.y() = vals[5];
          q.z() = vals[6];
          q.normalize();
          break;
        }
      }

      Eigen::Matrix4d T;
      Eigen::Vector3d p(vals[0], vals[1], vals[2]);
      beam::QuaternionAndTranslationToTransformMatrix(q, p, T);
      poses_.push_back(T);

      ros::Time time_stamp = time_start + duration;
      time_stamps_.push_back(time_stamp);
    }
  }
}

void Poses::LoadLoopClosedPaths(const std::string& bag_file_path,
                                const std::string& topic_loop_closed,
                                const std::string& topic_high_rate) {
  time_stamps_.clear();
  poses_.clear();

  boost::filesystem::path p(bag_file_path);
  bag_name_ = p.stem().string();

  // open bag
  rosbag::Bag bag;
  BEAM_INFO("Opening bag: {}", bag_file_path);
  bag.open(bag_file_path, rosbag::bagmode::Read);

  // load loop closed poses
  rosbag::View view_loop_closed(bag, rosbag::TopicQuery(topic_loop_closed),
                                ros::TIME_MIN, ros::TIME_MAX, true);
  if (view_loop_closed.size() == 0){
    BEAM_ERROR("No loop closed poses in bag");
    return;
  }
  
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
  utils::PathMsgToPoses(*path_msg, loop_closed_poses, fixed_frame_,
                        moving_frame_);

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
                                                 fixed_frame_, moving_frame_);
  }
  BEAM_INFO("Overrode {} duplicate poses.", num_duplicate_poses);

  if (loop_closed_poses.empty() && high_rate_poses.empty()) {
    BEAM_ERROR("No poses read.");
    return;
  } else if (loop_closed_poses.empty()) {
    BEAM_ERROR("No loop closed poses read, using high rate poses only.");
    utils::PoseMapToTimeAndPoseVecs(high_rate_poses, poses_, time_stamps_);
  } else if (high_rate_poses.empty()) {
    BEAM_ERROR("No high rate poses read, using loop closed poses only.");
    utils::PoseMapToTimeAndPoseVecs(loop_closed_poses, poses_, time_stamps_);
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
      time_stamps_.push_back(new_stamp);
      poses_.push_back(T_WORLDCORR_BASELINKHR);
    }
  }

  // check frames have been set, if not set defaults
  if (fixed_frame_.empty()) { fixed_frame_ = "odom"; }
  if (moving_frame_.empty()) { moving_frame_ = "base_link"; }
  BEAM_INFO("Done loading poses from Bag. Saved {} total poses.",
            poses_.size());
}

void Poses::LoadLoopClosedPathsInterpolated(
    const std::string& bag_file_path, const std::string& topic_loop_closed,
    const std::string& topic_high_rate) {
  time_stamps_.clear();
  poses_.clear();

  boost::filesystem::path p(bag_file_path);
  bag_name_ = p.stem().string();

  // open bag
  rosbag::Bag bag;
  BEAM_INFO("Opening bag: {}", bag_file_path);
  bag.open(bag_file_path, rosbag::bagmode::Read);

  // load loop closed (LC) poses
  rosbag::View view_loop_closed(bag, rosbag::TopicQuery(topic_loop_closed),
                                ros::TIME_MIN, ros::TIME_MAX, true);
  if (view_loop_closed.size() == 0){
    BEAM_ERROR("No loop closed poses in bag");
    return;
  }

  // first, check if LC topic is path
  BEAM_INFO("Loading loop closed path messages from topic {}",
            topic_loop_closed);
  for (auto iter = view_loop_closed.begin(); iter != view_loop_closed.end();
       iter++) {
    auto path_msg_test = iter->instantiate<nav_msgs::Path>();
    if (path_msg_test == NULL) {
      BEAM_CRITICAL("Loop closed trajectory message in bag is not of type "
                    "nav_msgs::Path");
      throw std::runtime_error{"Invalid message type for input message topic."};
    }
    break;
  }

  // get last LC path message
  boost::shared_ptr<nav_msgs::Path> path_msg_LC;
  for (auto iter = view_loop_closed.begin(); iter != view_loop_closed.end();
       iter++) {
    path_msg_LC = iter->instantiate<nav_msgs::Path>();
  }
  if (path_msg_LC == NULL) {
    throw std::runtime_error{"Cannot instantiate path msg."};
  }

  // convert last LC path to tf tree
  pose_map_type loop_closed_poses;
  utils::PathMsgToPoses(*path_msg_LC, loop_closed_poses, fixed_frame_,
                        moving_frame_);

  // next, load high rate (HR) path
  rosbag::View view_high_rate(bag, rosbag::TopicQuery(topic_high_rate),
                              ros::TIME_MIN, ros::TIME_MAX, true);

  // first, check if HR topic is odom or path
  BEAM_INFO("Loading high rate odom/path messages from topic {}",
            topic_high_rate);
  bool is_HR_odom{true};
  boost::shared_ptr<nav_msgs::Odometry> odom_msg_HR;
  boost::shared_ptr<nav_msgs::Path> path_msg_HR;
  for (auto iter = view_high_rate.begin(); iter != view_high_rate.end();
       iter++) {
    odom_msg_HR = iter->instantiate<nav_msgs::Odometry>();
    if (odom_msg_HR == NULL) {
      is_HR_odom = false;
    } else {
      break;
    }

    path_msg_HR = iter->instantiate<nav_msgs::Path>();
    if (path_msg_HR == NULL) {
      BEAM_CRITICAL("High rate trajectory message in bag is not of type "
                    "nav_msgs::Odometry or nav_msgs::Path");
      throw std::runtime_error{"Invalid message type for input message topic."};
    }
    break;
  }

  // convert HR msgs to tf tree
  pose_map_type high_rate_poses;
  int num_duplicate_poses{0};
  if (is_HR_odom) {
    // get HR odometry
    for (auto iter = view_high_rate.begin(); iter != view_high_rate.end();
         iter++) {
      odom_msg_HR = iter->instantiate<nav_msgs::Odometry>();
      if (odom_msg_HR == NULL) {
        throw std::runtime_error{"Cannot instantiate odom msg."};
      }
      utils::OdomMsgToPoses(*odom_msg_HR, high_rate_poses, fixed_frame_,
                            moving_frame_);
    }
  } else {
    // get all HR paths
    for (auto iter = view_high_rate.begin(); iter != view_high_rate.end();
         iter++) {
      path_msg_HR = iter->instantiate<nav_msgs::Path>();
      if (path_msg_HR == NULL) {
        throw std::runtime_error{"Cannot instantiate path msg."};
      }
      num_duplicate_poses += utils::PathMsgToPoses(
          *path_msg_HR, high_rate_poses, fixed_frame_, moving_frame_);
    }
    BEAM_INFO("Overrode {} duplicate poses.", num_duplicate_poses);
  }

  // check LC and HR pose maps
  if (loop_closed_poses.empty() && high_rate_poses.empty()) {
    BEAM_ERROR("No poses read.");
    return;
  } else if (loop_closed_poses.empty()) {
    BEAM_ERROR("No loop closed poses read, using high rate poses only.");
    utils::PoseMapToTimeAndPoseVecs(high_rate_poses, poses_, time_stamps_);
    // check frames have been set, if not set defaults
    if (fixed_frame_.empty()) { fixed_frame_ = "odom"; }
    if (moving_frame_.empty()) { moving_frame_ = "base_link"; }
    BEAM_INFO("Done loading poses from Bag. Saved {} total poses.",
              poses_.size());
    return;
  } else if (high_rate_poses.empty()) {
    BEAM_ERROR("No high rate poses read, using loop closed poses only.");
    utils::PoseMapToTimeAndPoseVecs(loop_closed_poses, poses_, time_stamps_);
    // check frames have been set, if not set defaults
    if (fixed_frame_.empty()) { fixed_frame_ = "odom"; }
    if (moving_frame_.empty()) { moving_frame_ = "base_link"; }
    BEAM_INFO("Done loading poses from Bag. Saved {} total poses.",
              poses_.size());
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
  Eigen::Matrix4d T_WORLDCORR_WORLDEST;
  beam_calibration::TfTree transforms_HR;
  auto iter_HR_prev = high_rate_poses.begin();
  auto iter_LC = loop_closed_poses.begin();
  for (auto iter_HR = high_rate_poses.begin(); iter_HR != high_rate_poses.end();
       iter_HR++) {
    // get time of curent HR and LC poses
    const uint64_t& t_HR = iter_HR->first;
    const uint64_t& t_LC = iter_LC->first;

    // if time of current HR pose is greater or equal to LC pose, then add
    // correction and increment LC iter
    if (t_HR >= t_LC && iter_LC != loop_closed_poses.end()) {
      // get stamp of LC pose
      ros::Time stamp_LC;
      stamp_LC.fromNSec(t_LC);

      // if time of current HR pose is greater than LC pose, then interpolate a
      // HR pose at time of LC pose
      Eigen::Matrix4d T_WORLDEST_BASELINKHR;
      if (t_HR > t_LC) {
        // add current and previous transforms to buffer
        const Eigen::Affine3d T_WB_HR_prev(iter_HR_prev->second);
        const Eigen::Affine3d T_WB_HR_curr(iter_HR->second);

        ros::Time stamp_curr;
        stamp_curr.fromNSec(t_HR);

        ros::Time stamp_prev;
        stamp_prev.fromNSec(iter_HR_prev->first);

        transforms_HR.AddTransform(T_WB_HR_prev, "W", "B", stamp_prev);
        transforms_HR.AddTransform(T_WB_HR_curr, "W", "B", stamp_curr);

        // get transform at time of LC pose
        T_WORLDEST_BASELINKHR =
            transforms_HR.GetTransformEigen("W", "B", stamp_LC).matrix();
      } else {
        T_WORLDEST_BASELINKHR = iter_HR->second;
      }

      // get correction at time of LC pose
      const Eigen::Matrix4d& T_WORLD_BASELINKLC = iter_LC->second;
      T_WORLDCORR_WORLDEST =
          T_WORLD_BASELINKLC * beam::InvertTransform(T_WORLDEST_BASELINKHR);

      // add correction to tf tree
      Eigen::Affine3d T(T_WORLDCORR_WORLDEST);
      corrections.AddTransform(T, "WORLD_CORRECTED", "WORLD_ESTIMATED",
                               stamp_LC);

      // increment LC iterator
      iter_LC++;
    }
    // reset previous HR iterator
    iter_HR_prev = iter_HR;
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
    time_stamps_.push_back(stamp_HR);
    poses_.push_back(T_WORLDCORR_BASELINKHR);
  }
}

void Poses::LoadFromBAG(const std::string& bag_file_path,
                        const std::string& topic) {
  time_stamps_.clear();
  poses_.clear();

  boost::filesystem::path p(bag_file_path);
  bag_name_ = p.stem().string();

  // open bag
  rosbag::Bag bag;
  BEAM_INFO("Opening bag: {}", bag_file_path);
  bag.open(bag_file_path, rosbag::bagmode::Read);

  rosbag::View view(bag, rosbag::TopicQuery(topic), ros::TIME_MIN,
                    ros::TIME_MAX, true);
  if (view.size() == 0){
    BEAM_ERROR("No messages with topic: {} ", topic);
    return;
  }

  auto iter = view.begin();
  auto odom_msg = iter->instantiate<nav_msgs::Odometry>();
  auto path_msg = iter->instantiate<nav_msgs::Path>();
  auto geo_msg = iter->instantiate<geometry_msgs::TransformStamped>();

  if (odom_msg != NULL) {
    BEAM_INFO("Loading odom messages from bag");
    for (auto iter = view.begin(); iter != view.end(); iter++) {
      odom_msg = iter->instantiate<nav_msgs::Odometry>();
      if (odom_msg == NULL) {
        throw std::runtime_error{"Cannot instantiate odometry msg."};
      }

      if (fixed_frame_.empty()) { fixed_frame_ = odom_msg->header.frame_id; }
      if (moving_frame_.empty()) { moving_frame_ = odom_msg->child_frame_id; }
      time_stamps_.push_back(odom_msg->header.stamp);
      Eigen::Affine3d T_FIXED_MOVING;
      Eigen::fromMsg(odom_msg->pose.pose, T_FIXED_MOVING);
      poses_.push_back(T_FIXED_MOVING.matrix());
    }
  } else if (path_msg != NULL) {
    BEAM_INFO("Loading path messages from bag");
    // get last path message
    auto iter = view.begin();
    std::advance(iter, view.size() - 1);
    path_msg = iter->instantiate<nav_msgs::Path>();

    if (path_msg == NULL) {
      throw std::runtime_error{"Cannot instantiate path msg."};
    }

    // convert to poses
    utils::PathMsgToPoses(*path_msg, poses_, time_stamps_, fixed_frame_,
                          moving_frame_);
  } else if (geo_msg != NULL) {
    BEAM_INFO("Loading geometry messages from bag");
    for (auto iter = view.begin(); iter != view.end(); iter++) {
      geo_msg = iter->instantiate<geometry_msgs::TransformStamped>();
      if (geo_msg == NULL) {
        throw std::runtime_error{"Cannot instantiate geometry msg."};
      }

      if (fixed_frame_.empty()) { fixed_frame_ = geo_msg->header.frame_id; }
      if (moving_frame_.empty()) { moving_frame_ = geo_msg->child_frame_id; }
      time_stamps_.push_back(geo_msg->header.stamp);
      Eigen::Matrix4d T_FIXED_MOVING;
      Eigen::Quaterniond q;
      Eigen::fromMsg(geo_msg->transform.rotation, q);
      Eigen::Matrix3d R = q.toRotationMatrix();
      T_FIXED_MOVING.block(0, 0, 3, 3) = R;
      T_FIXED_MOVING(0, 3) = geo_msg->transform.translation.x;
      T_FIXED_MOVING(1, 3) = geo_msg->transform.translation.y;
      T_FIXED_MOVING(2, 3) = geo_msg->transform.translation.z;
      poses_.push_back(T_FIXED_MOVING.matrix());
    }
  } else {
    BEAM_ERROR(
        "Cannot instantiate message. Invalid message type. Options: "
        "nav_msgs::Odometry, nav_msgs::Path, geometry_msgs::TransformStamped");
    throw std::runtime_error{"Cannot instantiate message."};
  }

  if (fixed_frame_.empty()) { fixed_frame_ = "odom"; }
  if (moving_frame_.empty()) { moving_frame_ = "base_link"; }
  BEAM_INFO("Done loading poses from Bag. Saved {} total poses.",
            poses_.size());
}

void Poses::LoadFromPCD(const std::string& input_pose_file_path) {
  time_stamps_.clear();
  poses_.clear();

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
    poses_.push_back(T);
    time_stamps_.push_back(t);
  }
}

std::ofstream Poses::CreateFile(const std::string& output_path,
                                const std::string& extension) const {
  std::string output_file = GetOutputFileName(output_path, extension);
  BEAM_INFO("Saving poses to file: {}", output_file);
  return std::ofstream(output_file);
}

std::string Poses::GetOutputFileName(const std::string& output_path,
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
    output_file = output_path;
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

  return output_file;
}

bool Poses::CheckPoses() const {
  if (poses_.size() != time_stamps_.size()) {
    BEAM_CRITICAL("Number of time stamps not equal to number of poses. Not "
                  "outputting to pose file.");
    throw std::runtime_error{"Number of time stamps not equal to number of "
                             "poses. Cannot create pose file."};
    return false;
  } else {
    return true;
  }
}

} // namespace beam_mapping
