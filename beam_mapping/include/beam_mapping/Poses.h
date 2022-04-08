/** @file
 * @ingroup mapping
 */

#pragma once

#include <fstream>

#include <Eigen/Dense>
#include <ros/time.h>

#include <beam_utils/math.h>

namespace beam_mapping {
/** @addtogroup mapping
 *  @{ */

enum format_type { Type1 = 1, Type2 };

/**
 * @brief class for map builder
 */
class Poses {
public:
  /**
   * @brief default constructor
   */
  Poses() = default;

  /**
   * @brief Default destructor
   */
  ~Poses() = default;

  /**
   * @brief clears all content
   */
  void Clear();

  /**
   * @brief for setting bag_name to pose file
   * @param bag_name
   */
  void SetBagName(const std::string& bag_name);

  /**
   * @brief for getting bag_name
   * @return bag_name
   */
  std::string GetBagName() const;

  /**
   * @brief for setting pose_file_date to pose file
   * @param pose_file_date
   */
  void SetPoseFileDate(const std::string& pose_file_date);

  /**
   * @brief for getting pose_file_date
   * @return pose_file_date
   */
  std::string GetPoseFileDate() const;

  /**
   * @brief for setting fixed_frame to pose file
   * @param fixed_frame
   */
  void SetFixedFrame(const std::string& fixed_frame);

  /**
   * @brief for getting fixed_frame
   * @return fixed_frame
   */
  std::string GetFixedFrame() const;

  /**
   * @brief for setting moving_frame to pose file
   * @param moving_frame
   */
  void SetMovingFrame(const std::string& moving_frame);

  /**
   * @brief for getting moving_frame
   * @return moving_frame
   */
  std::string GetMovingFrame() const;

  /**
   * @brief for setting time stamps
   * @param time_stamps
   */
  void SetTimeStamps(const std::vector<ros::Time>& time_stamps);

  /**
   * @brief for getting the time stamps
   * @return time_stamps
   */
  std::vector<ros::Time> GetTimeStamps() const;

  /**
   * @brief for adding a single time stamp
   * @param time_stamp
   */
  void AddSingleTimeStamp(const ros::Time& time_stamp);

  /**
   * @brief for setting poses vector<T_FIXED_MOVING>
   * @param _poses transforms from moving frame to fixed frame
   */
  void SetPoses(const std::vector<Eigen::Matrix4d, beam::AlignMat4d>& poses);

  /**
   * @brief for getting the poses
   * @return vector<T_FIXED_MOVING> poses that transform from moving frame to
   * fixed frame
   */
  std::vector<Eigen::Matrix4d, beam::AlignMat4d> GetPoses() const;

  /**
   * @brief for adding a single pose
   * @param pose transform from moving frame to fixed frame
   */
  void AddSinglePose(const Eigen::Matrix4d& T_FIXED_MOVING);

  /**
   * @brief load from file. This will lookup the extension and call the
   * appropriate load function.
   * @param input_pose_file_path full path to pose file. File extensions
   * supported include: .ply, .json, .txt, .pcd
   * @param format_type int specifying i/o format type. Certain load
   * functions will have the option to load a file with different formats. See
   * load function documentation for details on the meaning of integer value
   * @return false if file type is incorrect
   */
  bool LoadFromFile(const std::string& input_pose_file_path,
                    int format_type = format_type::Type1);

  /**
   * @brief writes the pose file to the specified directory with file type
   * specified by the user. If a directory is given (i.e. ending in /) the file
   * will be named: "poses_file_date"_poses.file_type. If a full filename is
   * given (i.e. /path/filename.file_type) it will keep that name.
   * @param file_type specifies the file type to which poses are written. File
   * types supported include: "JSON", "PLY", "TXT"
   * @param format_type int specifying i/o format type. Certain load
   * functions will have the option to load a file with different formats. See
   * README for documentation
   */
  bool WriteToFile(const std::string& output_dir, const std::string& file_type,
                   int format_type = format_type::Type1);

  /**
   * @brief writes the pose file to the specified directory as JSON type
   * @param output_dir full path to directory at which to save pose file
   */
  void WriteToJSON(const std::string& output_dir) const;

  /**
   * @brief loads the pose file in JSON format
   * @param input_pose_file_path full path to pose file
   */
  void LoadFromJSON(const std::string& input_pose_file_path);

  /**
   * @brief writes the pose file to the specified directory as TXT type
   * @param output_dir full path to directory at which to save pose file
   * @param format_type int specifying i/o format type.
   */
  void WriteToTXT(const std::string& output_dir,
                  int format_type = format_type::Type1) const;

  /**
   * @brief loads the pose file in txt format
   * @param input_pose_file_path full path to pose file
   * @param format_type int specifying i/o format type.
   */
  void LoadFromTXT(const std::string& input_pose_file_path,
                   int format_type = format_type::Type1);

  /**
   * @brief writes the pose file to the specified directory as PLY type
   * @param output_dir full path to directory at which to save pose file
   * @param format_type int specifying i/o format type.
   */
  void WriteToPLY(const std::string& output_dir,
                  int format_type = format_type::Type1) const;

  /**
   * @brief loads the pose file in PLY format. This checks the header comment
   * orientation_type and loads PLY format in either "Type1" or "Type2"
   * format, overriding format_type
   * @param input_pose_file_path full path to pose file
   * @param format_type int specifying i/o format type.
   */
  void LoadFromPLY(const std::string& input_pose_file_path,
                   int format_type = format_type::Type1);

  /**
   * @brief loads the pose file in PCD format
   * @param input_pose_file_path full path to pose file
   */
  void LoadFromPCD(const std::string& input_pose_file_path);

  /**
   * @brief loads the pose object using the odometry topic or path topic from a
   * bag file. If a path topic is supplied, this will take the final path
   * message in the bag.
   * @param bag_file_path full path to bag file
   * @param topic
   */
  void LoadFromBAG(const std::string& bag_file_path, const std::string& topic);

  /**
   * @brief Load two paths and combined into a single set of poses.
   * This takes one path which contains low rate poses which have been loop
   * closed, and one path which contains high rate poses that have not been
   * corrected for loop closure, and it combines them into a full loop closed
   * set of poses. Note: it expects the topic_loop_closed to contain all loop
   * closed poses in the final message, and the high rate topic is expected to
   * have different poses in each message
   * @param topic_loop_closed
   * @param topic_high_rate
   */
  void LoadLoopClosedPaths(const std::string& bag_file_path,
                           const std::string& topic_loop_closed,
                           const std::string& topic_high_rate);

  /**
   * @brief Same as LoadLoopClosedPaths, but interpolates the corrections at
   * each new pose. This makes sure the corrected trajectory is continuous
   * @param topic_loop_closed
   * @param topic_high_rate
   */
  void LoadLoopClosedPathsInterpolated(const std::string& bag_file_path,
                                       const std::string& topic_loop_closed,
                                       const std::string& topic_high_rate);

private:
  /**
   * @brief this is a helper function to create files to write to (e.g., .txt,
   * .ply). The goal of this is to make writing to a file more robust to user
   * input. Here is the logic:
   *
   *  - First we find the directory and make sure it exists, if not, we try to
   *    create the directory.
   *  - If output path has a file extension at the end, then the file will be
   *    nammed according to the exact input
   *  - If the output_path ends in '/' then we add poses.extension
   *  - If the output_path ends in something else, we post-fix with
   *    _poses.extension
   */
  std::ofstream CreateFile(const std::string& output_path,
                           const std::string& extension) const;

  /**
   * @brief converts tokens, seperated by a common deliminator, from an input
   * string into a vector of numeric values
   * @param deliminator deliminator seperating tokens
   * @param input_string input string to parse
   * @param values vector containing tokens as numeric values
   * @return true if values is non-empty
   */
  bool PopulateValues(const std::string& deliminator, std::string& input_string,
                      std::vector<double>& values);

  /**
   * @brief ensures that the numer of poses matches the number of time stamps
   */
  bool CheckPoses() const;

  std::vector<ros::Time> time_stamps_;
  std::vector<Eigen::Matrix4d, beam::AlignMat4d> poses_;
  std::string bag_name_;
  std::string pose_file_date_;
  std::string fixed_frame_;
  std::string moving_frame_;
};

/** @} group mapping */

} // namespace beam_mapping
