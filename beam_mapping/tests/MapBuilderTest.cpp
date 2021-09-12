#define CATCH_CONFIG_MAIN
#include "beam_mapping/MapBuilder.h"
#include "beam_utils/math.h"
#include <boost/filesystem.hpp>
#include <catch2/catch.hpp>

std::string GetFullFile(std::string current_rel_path,
                        std::string new_rel_path) {
  std::string path = __FILE__;
  path.erase(path.end() - current_rel_path.length(), path.end());
  path += new_rel_path;
  return path;
}

TEST_CASE("Testing map building with JSON") {
  std::string current_file_rel{"/tests/MapBuilderTest.cpp"};
  std::string config_file_rel{
      "/tests/test_data/MapBuilderTests/TestConfigJSON.json"};
  std::string pose_file_rel{
      "/tests/test_data/MapBuilderTests/TestPosesJSON.json"};
  std::string bag_file_rel{"/tests/test_data/MapBuilderTests/TestBagJSON.bag"};
  std::string extrinsics_file_rel{
      "/tests/test_data/MapBuilderTests/extrinsics.json"};
  std::string output_dir_rel{"/tests/test_data/MapBuilderTests/tmp/"};

  std::string config_file_path = GetFullFile(current_file_rel, config_file_rel);
  std::string pose_file_path = GetFullFile(current_file_rel, pose_file_rel);
  std::string bag_file_path = GetFullFile(current_file_rel, bag_file_rel);
  std::string extrinsics_file_path =
      GetFullFile(current_file_rel, extrinsics_file_rel);
  std::string output_dir_path = GetFullFile(current_file_rel, output_dir_rel);

  beam_mapping::MapBuilder map_builder(config_file_path);
  map_builder.OverrideBagFile(bag_file_path);
  map_builder.OverridePoseFile(pose_file_path);
  map_builder.OverrideExtrinsicsFile(extrinsics_file_path);
  map_builder.OverrideOutputDir(output_dir_path);
  boost::filesystem::create_directory(output_dir_path);
  REQUIRE_NOTHROW(map_builder.BuildMap());
  boost::filesystem::remove_all(output_dir_path);
}

TEST_CASE("Testing map building with PLY") {
  std::string current_file_rel{"/tests/MapBuilderTest.cpp"};
  std::string config_file_rel{
      "/tests/test_data/MapBuilderTests/TestConfigPLY.json"};
  std::string pose_file_rel{
      "/tests/test_data/MapBuilderTests/TestPosesPLY.ply"};
  std::string bag_file_rel{"/tests/test_data/MapBuilderTests/TestBagPLY.bag"};
  std::string extrinsics_file_rel{
      "/tests/test_data/MapBuilderTests/extrinsics.json"};
  std::string output_dir_rel{"/tests/test_data/MapBuilderTests/tmp/"};

  std::string config_file_path = GetFullFile(current_file_rel, config_file_rel);
  std::string pose_file_path = GetFullFile(current_file_rel, pose_file_rel);
  std::string bag_file_path = GetFullFile(current_file_rel, bag_file_rel);
  std::string extrinsics_file_path =
      GetFullFile(current_file_rel, extrinsics_file_rel);
  std::string output_dir_path = GetFullFile(current_file_rel, output_dir_rel);

  beam_mapping::MapBuilder map_builder(config_file_path);
  map_builder.OverrideBagFile(bag_file_path);
  map_builder.OverridePoseFile(pose_file_path);
  map_builder.OverrideExtrinsicsFile(extrinsics_file_path);
  map_builder.OverrideOutputDir(output_dir_path);
  std::string moving_frame = "vvlp_link";
  std::string fixed_frame = "odom";
  map_builder.SetPosesMovingFrame(moving_frame);
  map_builder.SetPosesFixedFrame(fixed_frame);
  boost::filesystem::create_directory(output_dir_path);
  REQUIRE_NOTHROW(map_builder.BuildMap());
  boost::filesystem::remove_all(output_dir_path);
}

TEST_CASE("Testing map building with PCD") {
  std::string current_file_rel{"/tests/MapBuilderTest.cpp"};
  std::string config_file_rel{
      "/tests/test_data/MapBuilderTests/TestConfigPCD.json"};
  std::string pose_file_rel{
      "/tests/test_data/MapBuilderTests/TestPosesPCD.pcd"};
  std::string bag_file_rel{"/tests/test_data/MapBuilderTests/TestBagPCD.bag"};
  std::string extrinsics_file_rel{
      "/tests/test_data/MapBuilderTests/extrinsics.json"};
  std::string output_dir_rel{"/tests/test_data/MapBuilderTests/tmp/"};

  std::string config_file_path = GetFullFile(current_file_rel, config_file_rel);
  std::string pose_file_path = GetFullFile(current_file_rel, pose_file_rel);
  std::string bag_file_path = GetFullFile(current_file_rel, bag_file_rel);
  std::string extrinsics_file_path =
      GetFullFile(current_file_rel, extrinsics_file_rel);
  std::string output_dir_path = GetFullFile(current_file_rel, output_dir_rel);

  beam_mapping::MapBuilder map_builder(config_file_path);
  map_builder.OverrideBagFile(bag_file_path);
  map_builder.OverridePoseFile(pose_file_path);
  map_builder.OverrideExtrinsicsFile(extrinsics_file_path);
  map_builder.OverrideOutputDir(output_dir_path);
  std::string moving_frame = "base_link";
  std::string fixed_frame = "odom";
  map_builder.SetPosesMovingFrame(moving_frame);
  map_builder.SetPosesFixedFrame(fixed_frame);
  boost::filesystem::create_directory(output_dir_path);
  REQUIRE_NOTHROW(map_builder.BuildMap());
  boost::filesystem::remove_all(output_dir_path);
}

TEST_CASE("Testing map building from Bag odometry") {
  std::string current_file_rel{"/tests/MapBuilderTest.cpp"};
  std::string config_file_rel{
      "/tests/test_data/MapBuilderTests/TestConfigBAG.json"};
  std::string pose_file_rel{"/tests/test_data/MapBuilderTests/"};
  std::string bag_file_rel{"/tests/test_data/MapBuilderTests/TestBagBAG.bag"};
  std::string extrinsics_file_rel{
      "/tests/test_data/MapBuilderTests/extrinsics.json"};
  std::string output_dir_rel{"/tests/test_data/MapBuilderTests/tmp/"};

  std::string config_file_path = GetFullFile(current_file_rel, config_file_rel);
  std::string pose_file_path = GetFullFile(current_file_rel, pose_file_rel);
  std::string bag_file_path = GetFullFile(current_file_rel, bag_file_rel);
  std::string extrinsics_file_path =
      GetFullFile(current_file_rel, extrinsics_file_rel);
  std::string output_dir_path = GetFullFile(current_file_rel, output_dir_rel);

  beam_mapping::Poses bag_poses;
  std::string bag_poses_file_path =
      "/home/alex/robots/data/ig/ig_scan_2019-02-13-19-44-24-loam.bag";
  if (!boost::filesystem::exists(bag_poses_file_path)) {
    BEAM_ERROR("Invalid bag file path, file does not exist. Ensure file is "
               "downloaded locally on your machine. Exiting Test. Input: {}",
               bag_poses_file_path);
    return;
  }

  std::string topic = "/ig/loam/lidar_odom";
  bag_poses.LoadFromBAG(bag_poses_file_path, topic);
  bag_poses.WriteToJSON(pose_file_path);
  beam_mapping::MapBuilder map_builder(config_file_path);
  map_builder.OverrideBagFile(bag_poses_file_path);
  map_builder.OverridePoseFile(pose_file_path + "_poses.json");
  map_builder.OverrideExtrinsicsFile(extrinsics_file_path);
  map_builder.OverrideOutputDir(output_dir_path);
  boost::filesystem::create_directory(output_dir_path);
  REQUIRE_NOTHROW(map_builder.BuildMap());
  boost::filesystem::remove_all(output_dir_path);
  boost::filesystem::remove_all(pose_file_path + "_poses.json");
}
