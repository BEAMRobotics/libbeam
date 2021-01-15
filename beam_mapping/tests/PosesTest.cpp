#define CATCH_CONFIG_MAIN
#include "beam_mapping/Poses.h"
#include "beam_utils/math.hpp"
#include <boost/filesystem.hpp>
#include <catch2/catch.hpp>

std::string GetFullFile(std::string current_rel_path,
                        std::string new_rel_path) {
  std::string path = __FILE__;
  path.erase(path.end() - current_rel_path.length(), path.end());
  path += new_rel_path;
  return path;
}

TEST_CASE("Test JSON read and write functionality") {
  std::string pose_file_rel = "/tests/test_data/PosesTests/PosesTest.json";
  std::string current_file_rel = "/tests/PosesTest.cpp";
  std::string pose_file_path = GetFullFile(current_file_rel, pose_file_rel);

  beam_mapping::Poses poses_read;
  poses_read.LoadFromJSON(pose_file_path);

  REQUIRE(poses_read.GetBagName() == "ig_scan_2019-02-13-19-44-24.bag");
  REQUIRE(poses_read.GetPoseFileDate() == "2019_4_23_10_1_44");
  REQUIRE(poses_read.GetFixedFrame() == "odom");
  REQUIRE(poses_read.GetMovingFrame() == "vvlp_link");

  beam::Mat4 T1, T2, T3;
  T1 << 1, 5.973908991266297e-17, 1.060206815998276e-16, 2.408428785093423e-19,
      -5.973908887784716e-17, 1, -2.655218167731045e-17, -3.961611144079579e-18,
      -1.060206819694939e-16, 2.655218081096116e-17, 1, -7.551404631739263e-18,
      0, 0, 0, 1;
  T2 << 0.9860362112117617, 0.1503750243106158, -0.07155377168772267,
      15.82236312877207, -0.1492502985015648, 0.988579716900445,
      0.02084446521823662, -1.839718858486883, 0.07387109432214155,
      -0.009873975725219933, 0.9972189157988478, 0.4227022479978343, 0, 0, 0, 1;
  T3 << 0.2114755644436612, -0.9678982915426839, -0.1358343950256415,
      23.73990045897456, 0.9773086563828571, 0.2076899137042982,
      0.04162559194330412, -6.664018422068914, -0.01207790554542605,
      -0.1415549256445998, 0.9898567205527172, 1.292950254770442, 0, 0, 0, 1;

  REQUIRE(poses_read.GetPoses()[0].matrix() == T1);
  REQUIRE(poses_read.GetPoses()[1].matrix() == T2);
  REQUIRE(poses_read.GetPoses()[2].matrix() == T3);

  ros::Time t1(1550105072, 919277056);
  ros::Time t2(1550105104, 118189056);
  ros::Time t3(1550105150, 526373888);

  REQUIRE(t1 == poses_read.GetTimeStamps()[0]);
  REQUIRE(t2 == poses_read.GetTimeStamps()[1]);
  REQUIRE(t3 == poses_read.GetTimeStamps()[2]);

  // Now output to new file, and repeat with new file. This should test the
  // write method
  std::string pose_output_path_rel, pose_output_path, pose_file_path2;
  pose_output_path_rel = "/tests/test_data/PosesTests/";
  pose_output_path = GetFullFile(current_file_rel, pose_output_path_rel);
  pose_file_path2 = pose_output_path + "2019_4_23_10_1_44_poses.json";

  poses_read.WriteToJSON(pose_output_path);
  beam_mapping::Poses poses_written;
  poses_written.LoadFromJSON(pose_file_path2);
  REQUIRE(poses_written.GetBagName() == "ig_scan_2019-02-13-19-44-24.bag");
  REQUIRE(poses_written.GetPoseFileDate() == "2019_4_23_10_1_44");
  REQUIRE(poses_written.GetFixedFrame() == "odom");
  REQUIRE(poses_written.GetMovingFrame() == "vvlp_link");
  REQUIRE(poses_written.GetPoses()[0].matrix() == T1);
  REQUIRE(poses_written.GetPoses()[1].matrix() == T2);
  REQUIRE(poses_written.GetPoses()[2].matrix() == T3);
  REQUIRE(t1 == poses_written.GetTimeStamps()[0]);
  REQUIRE(t2 == poses_written.GetTimeStamps()[1]);
  REQUIRE(t3 == poses_written.GetTimeStamps()[2]);
  boost::filesystem::remove(pose_file_path2);
}

TEST_CASE("Test PLY read and write functionality") {
  std::string pose_file_rel = "/tests/test_data/PosesTests/PosesTestKaarta.ply";
  std::string current_file_rel = "/tests/PosesTest.cpp";
  std::string pose_file_path = GetFullFile(current_file_rel, pose_file_rel);

  beam_mapping::Poses poses_read;
  poses_read.LoadFromPLY(pose_file_path);
  double time_2 = 1563850853.876633 + 0.403459;
  ros::Time time1(1563850853.876633), time2(time_2);
  REQUIRE(Approx(poses_read.poses[0](0, 3)).epsilon(0.00001) == 0.004724);
  REQUIRE(Approx(poses_read.poses[0](2, 3)).epsilon(0.00001) == 0.007737);
  REQUIRE(poses_read.time_stamps[0] == time1);
  REQUIRE(Approx(poses_read.poses[4](0, 3)).epsilon(0.00001) == 0.007888);
  REQUIRE(Approx(poses_read.poses[4](2, 3)).epsilon(0.00001) == 0.004562);
  REQUIRE(poses_read.time_stamps[4] == time2);

  // Now output to new file, and repeat with new file. This should test the
  // write method
  std::string pose_output_path_rel, pose_output_path, pose_file_path2;
  pose_output_path_rel = "/tests/test_data/PosesTests/";
  pose_output_path = GetFullFile(current_file_rel, pose_output_path_rel);
  pose_file_path2 = pose_output_path + "Mon Jul 22 23:00:53 2019 EDT_poses.ply";

  poses_read.WriteToPLY(pose_output_path);
  beam_mapping::Poses poses_written;
  poses_written.LoadFromPLY(pose_file_path2);
  REQUIRE(poses_written.GetPoseFileDate() == "Mon Jul 22 23:00:53 2019 EDT");
  REQUIRE(poses_written.GetPoses()[0](0, 3) == poses_read.poses[0](0, 3));
  REQUIRE(poses_written.GetPoses()[4](0, 3) == poses_read.poses[4](0, 3));
  REQUIRE(time1 == poses_written.GetTimeStamps()[0]);
  REQUIRE(time2 == poses_written.GetTimeStamps()[4]);
  boost::filesystem::remove(pose_file_path2);
}

TEST_CASE("Test TXT read and write functionality") {
  std::string current_file_rel = "/tests/PosesTest.cpp";
  std::string pose_file_rel = "/tests/test_data/PosesTests/PosesTest.txt";
  std::string pose_file_path = GetFullFile(current_file_rel, pose_file_rel);
  beam_mapping::Poses poses_read;
  INFO(pose_file_path);
  poses_read.LoadFromTXT(pose_file_path);
  ros::Time time1(100342.790712000), time2(100343.191518010);
  REQUIRE(Approx(poses_read.poses[0](0, 3)).epsilon(0.00001) == 0.432895);
  REQUIRE(Approx(poses_read.poses[0](2, 3)).epsilon(0.00001) == 0.003007);
  REQUIRE(poses_read.time_stamps[0] == time1);
  REQUIRE(Approx(poses_read.poses[4](0, 3)).epsilon(0.00001) == 0.619671);
  REQUIRE(Approx(poses_read.poses[4](2, 3)).epsilon(0.00001) == 0.013218);
  REQUIRE(poses_read.time_stamps[4] == time2);
  // Now output to new file, and repeat with new file. This should test the
  // write method
  std::string pose_output_path_rel, pose_output_path, pose_file_path2;
  pose_output_path_rel = "/tests/test_data/PosesTests/";
  pose_output_path = GetFullFile(current_file_rel, pose_output_path_rel);
  pose_file_path2 = pose_output_path + "_poses.txt";

  poses_read.WriteToTXT(pose_output_path);
  beam_mapping::Poses poses_written;
  poses_written.LoadFromTXT(pose_file_path2);
  REQUIRE(poses_written.GetPoses()[0](0, 3) == poses_read.poses[0](0, 3));
  REQUIRE(poses_written.GetPoses()[4](0, 3) == poses_read.poses[4](0, 3));
  REQUIRE(time1 == poses_written.GetTimeStamps()[0]);
  REQUIRE(time2 == poses_written.GetTimeStamps()[4]);
  boost::filesystem::remove(pose_file_path2);
}
