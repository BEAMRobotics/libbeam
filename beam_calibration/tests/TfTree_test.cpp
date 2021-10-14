#define CATCH_CONFIG_MAIN
#include "beam_calibration/TfTree.h"
#include "beam_utils/math.h"
#include <catch2/catch.hpp>
#include <boost/filesystem.hpp>

TEST_CASE("Test Tree building and retrieving") {
  int round_precision = 7;
  beam_calibration::TfTree Tree;
  Eigen::Matrix4d T_BASELINK_HVLP, T_X1_HVLP;
  Eigen::Affine3d TA_BASELINK_HVLP, TA_X1_HVLP, TA_BASELINK_X1_calc,
      TA_BASELINK_X1_lookup;

  T_BASELINK_HVLP << 1.00000, 0.00000, 0.00000, 0.2100, 0.00000, 1.00000,
      0.00000, 0.00000, 0.00000, 0.00000, 1.00000, 0.35200, 0.00000, 0.00000,
      0.00000, 1.00000;

  T_X1_HVLP << 0.00000, 0.00000, -1.00000, -0.0800, 0.00000, 1.00000, 0.00000,
      0.00000, 1.00000, 0.00000, 0.00000, -0.0400, 0.00000, 0.00000, 0.00000,
      1.00000;

  TA_BASELINK_HVLP.matrix() = T_BASELINK_HVLP;
  TA_X1_HVLP.matrix() = T_X1_HVLP;

  std::string to_frame1 = "BASELINK";
  std::string from_frame1 = "HVLP";
  Tree.AddTransform(TA_BASELINK_HVLP, to_frame1, from_frame1);
  Eigen::Affine3d TA_BASELINK_HVLP_lookup =
      Tree.GetTransformEigen(to_frame1, from_frame1);
  REQUIRE(TA_BASELINK_HVLP.matrix() ==
          beam::RoundMatrix(TA_BASELINK_HVLP_lookup.matrix(), round_precision));

  std::string to_frame2 = "X1";
  std::string from_frame2 = "HVLP";
  Tree.AddTransform(TA_X1_HVLP, to_frame2, from_frame2);
  Eigen::Affine3d T_X1_HVLP_lookup =
      Tree.GetTransformEigen(to_frame2, from_frame2);
  REQUIRE(T_X1_HVLP.matrix() ==
          beam::RoundMatrix(T_X1_HVLP_lookup.matrix(), round_precision));

  std::string to_frame3 = "BASELINK";
  std::string from_frame3 = "X1";
  TA_BASELINK_X1_calc.matrix() =
      TA_BASELINK_HVLP.matrix() * TA_X1_HVLP.matrix().inverse();
  TA_BASELINK_X1_lookup = Tree.GetTransformEigen(to_frame3, from_frame3);
  REQUIRE(beam::RoundMatrix(TA_BASELINK_X1_calc.matrix(), round_precision) ==
          beam::RoundMatrix(TA_BASELINK_X1_lookup.matrix(), round_precision));
  REQUIRE_THROWS(
      Tree.AddTransform(TA_BASELINK_X1_calc, to_frame3, from_frame3));

  std::string calib_date = "2018_12_20";
  Tree.SetCalibrationDate(calib_date);
  REQUIRE(Tree.GetCalibrationDate() == calib_date);
}

TEST_CASE("Testing multiple parent case") {
  beam_calibration::TfTree Tree1, Tree2;
  Eigen::Matrix4d T_HVLP_BASELINK, T_X1_HVLP, T_X1_IMU1;
  Eigen::Affine3d TA_HVLP_BASELINK, TA_X1_HVLP, TA_X1_IMU1, TA_IMU1_X1, T;

  T_HVLP_BASELINK << 1.00000, 0.00000, 0.00000, -0.2100, 0.00000, 1.00000,
      0.00000, 0.00000, 0.00000, 0.00000, 1.00000, -0.35200, 0.00000, 0.00000,
      0.00000, 1.00000;

  T_X1_HVLP << 0.00000, 0.00000, -1.00000, -0.0800, 0.00000, 1.00000, 0.00000,
      0.00000, 1.00000, 0.00000, 0.00000, -0.0400, 0.00000, 0.00000, 0.00000,
      1.00000;

  T_X1_IMU1 << 0.00000, 0.00000, -1.00000, 0.00000, -1.00000, 0.00000, 0.00000,
      0.00000, 0.00000, 1.00000, 0.00000, -0.0900, 0.00000, 0.00000, 0.00000,
      1.00000;

  TA_HVLP_BASELINK.matrix() = T_HVLP_BASELINK;
  TA_X1_HVLP.matrix() = T_X1_HVLP;
  TA_X1_IMU1.matrix() = T_X1_IMU1;
  TA_IMU1_X1 = TA_X1_IMU1.inverse();
  std::string to_frame1 = "HVLP";
  std::string from_frame1 = "BASELINK";
  std::string to_frame2 = "X1";
  std::string from_frame2 = "HVLP";
  std::string to_frame3 = "X1";
  std::string from_frame3 = "IMU1";

  Tree1.AddTransform(TA_HVLP_BASELINK, to_frame1, from_frame1);
  Tree1.AddTransform(TA_X1_HVLP, to_frame2, from_frame2);
  REQUIRE_NOTHROW(T = Tree1.GetTransformEigen(to_frame2, from_frame1));
  Tree1.AddTransform(TA_X1_IMU1, to_frame3, from_frame3);
  REQUIRE_NOTHROW(T = Tree1.GetTransformEigen(to_frame2, from_frame1));
  //
  // Tree2.AddTransform(TA_HVLP_BASELINK, to_frame1, from_frame1);
  // Tree2.AddTransform(TA_X1_HVLP, to_frame2, from_frame2);
  // REQUIRE_NOTHROW(T = Tree2.GetTransformEigen(to_frame2, from_frame1));
  // Tree2.AddTransform(TA_IMU1_X1, from_frame3, to_frame3);
  // REQUIRE_NOTHROW(T = Tree2.GetTransformEigen(to_frame2, from_frame1));
  // REQUIRE_NOTHROW(T = Tree2.GetTransformEigen(from_frame1, from_frame3));
}

TEST_CASE("Test multiple parent + interpolation for TransformStamped") {
  beam_calibration::TfTree Tree, TreeStatic;
  geometry_msgs::TransformStamped tf_msg1A, tf_msg1B, tf_msg1C, tf_msg2,
      tf_msg_lookup1, tf_msg_lookup2;

  ros::Time transform_time1(1, 0), transform_time2(2, 0),
      lookup_time1(1, 500000000), lookup_time2(3, 0);

  std::string from_frame = "hvlp_link";
  std::string to_frame1 = "X1_link";
  std::string to_frame2 = "IMU1_link";

  tf_msg1A.header.seq = 1;
  tf_msg1A.header.frame_id = to_frame1;
  tf_msg1A.child_frame_id = from_frame;
  tf_msg1A.header.stamp = transform_time1;
  tf_msg1A.transform.translation.x = 0;
  tf_msg1A.transform.translation.y = 0;
  tf_msg1A.transform.translation.z = 1;
  tf_msg1A.transform.rotation.x = 0;
  tf_msg1A.transform.rotation.y = 0;
  tf_msg1A.transform.rotation.z = 0;
  tf_msg1A.transform.rotation.w = 1;

  tf_msg1B.header.seq = 1;
  tf_msg1B.header.frame_id = to_frame1;
  tf_msg1B.child_frame_id = from_frame;
  tf_msg1B.header.stamp = transform_time2;
  tf_msg1B.transform.translation.x = 0;
  tf_msg1B.transform.translation.y = 0;
  tf_msg1B.transform.translation.z = 2;
  tf_msg1B.transform.rotation.x = 0;
  tf_msg1B.transform.rotation.y = 0;
  tf_msg1B.transform.rotation.z = 0;
  tf_msg1B.transform.rotation.w = 1;

  tf_msg2.header.seq = 2;
  tf_msg2.header.frame_id = to_frame2;
  tf_msg2.child_frame_id = from_frame;
  tf_msg2.header.stamp = transform_time1;
  tf_msg2.transform.translation.x = 1;
  tf_msg2.transform.translation.y = 1;
  tf_msg2.transform.translation.z = 1;
  tf_msg2.transform.rotation.x = 0;
  tf_msg2.transform.rotation.y = 0;
  tf_msg2.transform.rotation.z = 0;
  tf_msg2.transform.rotation.w = 1;

  Tree.AddTransform(tf_msg1A, false);
  Tree.AddTransform(tf_msg1B, false);
  REQUIRE_NOTHROW(tf_msg_lookup1 = Tree.GetTransformROS(to_frame1, from_frame,
                                                        lookup_time1));

  REQUIRE_THROWS(tf_msg_lookup2 =
                     Tree.GetTransformROS(to_frame1, from_frame, lookup_time2));

  REQUIRE(tf_msg_lookup1.transform.translation.z == 1.5);

  REQUIRE_NOTHROW(tf_msg_lookup2 = Tree.GetTransformROS(to_frame1, from_frame,
                                                        transform_time1));
  Tree.AddTransform(tf_msg2, false);
  REQUIRE_NOTHROW(tf_msg_lookup1 = Tree.GetTransformROS(to_frame2, from_frame,
                                                        transform_time1));
  TreeStatic.AddTransform(tf_msg1A, true);
  TreeStatic.AddTransform(tf_msg2, true);
  REQUIRE_NOTHROW(tf_msg_lookup2 = TreeStatic.GetTransformROS(
                      to_frame2, from_frame, transform_time1));
}

TEST_CASE("Test same dynamic transform case with same timestamp") {
  beam_calibration::TfTree Tree;
  geometry_msgs::TransformStamped tf_msg1, tf_msg2, tf_msg;
  ros::Time::init();
  ros::Time transform_time = ros::Time::now();

  std::string to_frame = "X1_link";
  std::string from_frame = "hvlp_link";

  tf_msg1.header.seq = 1;
  tf_msg1.header.frame_id = to_frame;
  tf_msg1.child_frame_id = from_frame;
  tf_msg1.header.stamp = transform_time;
  tf_msg1.transform.translation.x = 0;
  tf_msg1.transform.translation.y = 0;
  tf_msg1.transform.translation.z = 1;
  tf_msg1.transform.rotation.x = 0;
  tf_msg1.transform.rotation.y = 0;
  tf_msg1.transform.rotation.z = 0;
  tf_msg1.transform.rotation.w = 1;

  tf_msg2.header.seq = 1;
  tf_msg2.header.frame_id = to_frame;
  tf_msg2.child_frame_id = from_frame;
  tf_msg2.header.stamp = transform_time;
  tf_msg2.transform.translation.x = 0;
  tf_msg2.transform.translation.y = 0;
  tf_msg2.transform.translation.z = 1;
  tf_msg2.transform.rotation.x = 0;
  tf_msg2.transform.rotation.y = 0;
  tf_msg2.transform.rotation.z = 0;
  tf_msg2.transform.rotation.w = 1;

  Tree.AddTransform(tf_msg1);
  Tree.AddTransform(tf_msg2);
  REQUIRE_NOTHROW(
      tf_msg = Tree.GetTransformROS(to_frame, from_frame, transform_time));
}

TEST_CASE("Test loading tree from .json") {
  // create empty objects
  beam_calibration::TfTree Tree;
  Eigen::Matrix4d T_BASELINK_HVLP, T_X1_HVLP, T_BASELINK_HVLP_JSON, T_X1_HVLP_JSON,
      T_BASELINK_X1, T_BASELINK_X1_JSON;
  int round_precision = 7;

  // add true data
  std::string calib_date = "2018_12_20";
  T_BASELINK_HVLP << 1.00000, 0.00000, 0.00000, 0.2100, 0.00000, 1.00000,
      0.00000, 0.00000, 0.00000, 0.00000, 1.00000, 0.35200, 0.00000, 0.00000,
      0.00000, 1.00000;

  T_X1_HVLP << 0.00000, 0.00000, -1.00000, -0.0800, 0.00000, 1.00000, 0.00000,
      0.00000, 1.00000, 0.00000, 0.00000, -0.0400, 0.00000, 0.00000, 0.00000,
      1.00000;

  T_BASELINK_X1 = T_BASELINK_HVLP * T_X1_HVLP.inverse();

  // Load Tree from json
  std::string filename = "extrinsics.json";
  std::string file_location = __FILE__;
  file_location.erase(file_location.end() - 15, file_location.end());
  file_location += "test_data/";
  file_location += filename;
  Tree.LoadJSON(file_location);

  // look up transforms needed for tests
  std::string to_frame1 = "BASELINK";
  std::string from_frame1 = "HVLP";
  std::string to_frame2 = "X1";
  std::string from_frame2 = "HVLP";
  std::string to_frame3 = "BASELINK";
  std::string from_frame3 = "X1";

  T_BASELINK_HVLP_JSON =
      Tree.GetTransformEigen(to_frame1, from_frame1).matrix();
  T_X1_HVLP_JSON = Tree.GetTransformEigen(to_frame2, from_frame2).matrix();
  T_BASELINK_X1_JSON = Tree.GetTransformEigen(to_frame3, from_frame3).matrix();

  // perform tests
  REQUIRE(Tree.GetCalibrationDate() == calib_date);
  REQUIRE(beam::RoundMatrix(T_BASELINK_HVLP, round_precision) ==
          beam::RoundMatrix(T_BASELINK_HVLP_JSON, round_precision));
  REQUIRE(beam::RoundMatrix(T_X1_HVLP, round_precision) ==
          beam::RoundMatrix(T_X1_HVLP_JSON, round_precision));
  REQUIRE(beam::RoundMatrix(T_BASELINK_X1, round_precision) ==
          beam::RoundMatrix(T_BASELINK_X1_JSON, round_precision));
}
