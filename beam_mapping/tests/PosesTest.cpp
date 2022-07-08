#define CATCH_CONFIG_MAIN

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <catch2/catch.hpp>

#include <beam_mapping/Poses.h>
#include <beam_utils/filesystem.h>
#include <beam_utils/math.h>

std::string data_path_ =
    beam::LibbeamRoot() + "beam_mapping/tests/test_data/PosesTests/";

void CheckLoadWrite(const std::string& pose_file_path,
                    const std::string& file_type, int format_type = 1) {
  // load poses
  beam_mapping::Poses poses_read;
  poses_read.LoadFromFile(pose_file_path, format_type);
  auto transforms_read = poses_read.GetPoses();
  auto stamps_read = poses_read.GetTimeStamps();

  // write poses to new file and reload
  std::string output_ext = file_type;
  std::transform(output_ext.begin(), output_ext.end(), output_ext.begin(),
                 ::tolower);
  std::string pose_file_path_temp = data_path_ + "poses_temp." + output_ext;
  poses_read.WriteToFile(pose_file_path_temp, file_type, format_type);
  beam_mapping::Poses poses_written;
  poses_written.LoadFromFile(pose_file_path_temp, format_type);
  auto transforms_written = poses_written.GetPoses();
  auto stamps_written = poses_written.GetTimeStamps();

  // check sizes
  REQUIRE(transforms_written.size() == transforms_read.size());
  REQUIRE(stamps_written.size() == stamps_read.size());
  REQUIRE(stamps_read.size() == transforms_written.size());

  // check values are equal within a certain level of precision
  int round_precision = 5;
  size_t increment = 5;
  for (size_t i = 0; i < transforms_read.size(); i += increment) {
    REQUIRE(beam::RoundMatrix(transforms_written[i], round_precision) ==
            beam::RoundMatrix(transforms_read[i], round_precision));
    REQUIRE(std::abs(stamps_written[i].toSec() - stamps_read[i].toSec()) <
            0.000001);
  }
  boost::filesystem::remove(pose_file_path_temp);
}

TEST_CASE("Test JSON read and write functionality") {
  std::string pose_file_path = data_path_ + "PosesTest.json";
  CheckLoadWrite(pose_file_path, "JSON");

  beam_mapping::Poses poses_read;
  poses_read.LoadFromFile(pose_file_path);
  REQUIRE(poses_read.GetBagName() == "ig_scan_2019-02-13-19-44-24.bag");
  REQUIRE(poses_read.GetPoseFileDate() == "2019_4_23_10_1_44");
  REQUIRE(poses_read.GetFixedFrame() == "odom");
  REQUIRE(poses_read.GetMovingFrame() == "vvlp_link");
}

TEST_CASE(
    "Test TXT read and write functionality with pose format type: Type1") {
  std::string pose_file_path = data_path_ + "PosesTestType1.txt";
  CheckLoadWrite(pose_file_path, "TXT", beam_mapping::format_type::Type1);
}

TEST_CASE(
    "Test TXT read and write functionality with pose format type: Type2") {
  std::string pose_file_path = data_path_ + "PosesTestType2.txt";
  CheckLoadWrite(pose_file_path, "TXT", beam_mapping::format_type::Type2);
}

TEST_CASE(
    "Test PLY read and write functionality with pose format type: Type1") {
  std::string pose_file_path = data_path_ + "PosesTestType1.ply";
  CheckLoadWrite(pose_file_path, "PLY", beam_mapping::format_type::Type1);

  beam_mapping::Poses poses_read;
  poses_read.LoadFromFile(pose_file_path);
  REQUIRE(poses_read.GetBagName() == "test_bag.bag");
  REQUIRE(poses_read.GetPoseFileDate() == "Mon Jul 22 23:00:53 2019 EDT");
  REQUIRE(poses_read.GetFixedFrame() == "test_fixed_frame");
  REQUIRE(poses_read.GetMovingFrame() == "test_moving_frame");
}

TEST_CASE(
    "Test PLY read and write functionality with pose format type: Type2") {
  std::string pose_file_path = data_path_ + "PosesTestType2.ply";
  CheckLoadWrite(pose_file_path, "PLY", beam_mapping::format_type::Type2);

  beam_mapping::Poses poses_read;
  poses_read.LoadFromFile(pose_file_path);
  REQUIRE(poses_read.GetBagName() == "test_bag.bag");
  REQUIRE(poses_read.GetPoseFileDate() == "Mon Jul 22 23:00:53 2019 EDT");
  REQUIRE(poses_read.GetFixedFrame() == "test_fixed_frame");
  REQUIRE(poses_read.GetMovingFrame() == "test_moving_frame");
}
