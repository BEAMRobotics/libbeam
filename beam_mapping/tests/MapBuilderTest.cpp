#define CATCH_CONFIG_MAIN
#include "beam_mapping/MapBuilder.h"
#include "beam_utils/math.hpp"
#include <boost/filesystem.hpp>
#include <catch2/catch.hpp>

std::string GetFullFile(std::string current_rel_path, std::string new_rel_path){
  std::string path = __FILE__;
  path.erase(path.end() - current_rel_path.length(), path.end());
  path += new_rel_path;
  return path;
}

TEST_CASE("Testing map building with JSON") {
  std::string current_file_rel, config_file_rel, pose_file_rel, bag_file_rel,
              bag_file_path, pose_file_path, config_file_path,
              extrinsics_file_path, extrinsics_file_rel, output_dir_rel,
              output_dir_path;
  current_file_rel = "/tests/MapBuilderTest.cpp";
  config_file_rel = "/tests/test_data/MapBuilderTests/TestConfigJSON.json";
  pose_file_rel = "/tests/test_data/MapBuilderTests/TestPosesJSON.json";
  bag_file_rel = "/tests/test_data/MapBuilderTests/TestBagJSON.bag";
  extrinsics_file_rel = "/tests/test_data/MapBuilderTests/extrinsics.json";
  output_dir_rel = "/tests/test_data/MapBuilderTests/tmp/";

  config_file_path = GetFullFile(current_file_rel, config_file_rel);
  pose_file_path = GetFullFile(current_file_rel, pose_file_rel);
  bag_file_path = GetFullFile(current_file_rel, bag_file_rel);
  extrinsics_file_path = GetFullFile(current_file_rel, extrinsics_file_rel);
  output_dir_path = GetFullFile(current_file_rel, output_dir_rel);

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
  std::string current_file_rel, config_file_rel, pose_file_rel, bag_file_rel,
              bag_file_path, pose_file_path, config_file_path,
              extrinsics_file_path, extrinsics_file_rel, output_dir_rel,
              output_dir_path;
  current_file_rel = "/tests/MapBuilderTest.cpp";
  config_file_rel = "/tests/test_data/MapBuilderTests/TestConfigPLY.json";
  pose_file_rel = "/tests/test_data/MapBuilderTests/TestPosesPLY.ply";
  bag_file_rel = "/tests/test_data/MapBuilderTests/TestBagPLY.bag";
  extrinsics_file_rel = "/tests/test_data/MapBuilderTests/extrinsics.json";
  output_dir_rel = "/tests/test_data/MapBuilderTests/tmp/";

  config_file_path = GetFullFile(current_file_rel, config_file_rel);
  pose_file_path = GetFullFile(current_file_rel, pose_file_rel);
  bag_file_path = GetFullFile(current_file_rel, bag_file_rel);
  extrinsics_file_path = GetFullFile(current_file_rel, extrinsics_file_rel);
  output_dir_path = GetFullFile(current_file_rel, output_dir_rel);

  beam_mapping::MapBuilder map_builder(config_file_path);
  map_builder.OverrideBagFile(bag_file_path);
  map_builder.OverridePoseFile(pose_file_path);
  map_builder.OverrideExtrinsicsFile(extrinsics_file_path);
  map_builder.OverrideOutputDir(output_dir_path);
  std::string moving_frame = "hvlp_link";
  std::string fixed_frame = "odom";
  map_builder.SetPosesMovingFrame(moving_frame);
  map_builder.SetPosesFixedFrame(fixed_frame);
  boost::filesystem::create_directory(output_dir_path);
  REQUIRE_NOTHROW(map_builder.BuildMap());
  boost::filesystem::remove_all(output_dir_path);
}

TEST_CASE("Testing map building from Bag odometry") {
  std::string current_file_rel, config_file_rel, pose_file_rel, bag_file_rel,
              bag_file_path, pose_file_path, config_file_path,
              extrinsics_file_path, extrinsics_file_rel, output_dir_rel,
              output_dir_path;
  current_file_rel = "/tests/MapBuilderTest.cpp";
  config_file_rel = "/tests/test_data/MapBuilderTests/TestConfigBAG.json";
  pose_file_rel = "/tests/test_data/MapBuilderTests/";
  bag_file_rel = "/tests/test_data/MapBuilderTests/TestBagBAG.bag";
  extrinsics_file_rel = "/tests/test_data/MapBuilderTests/extrinsics.json";
  output_dir_rel = "/tests/test_data/MapBuilderTests/tmp/";

  config_file_path = GetFullFile(current_file_rel, config_file_rel);
  pose_file_path = GetFullFile(current_file_rel, pose_file_rel);
  bag_file_path = GetFullFile(current_file_rel, bag_file_rel);
  extrinsics_file_path = GetFullFile(current_file_rel, extrinsics_file_rel);
  output_dir_path = GetFullFile(current_file_rel, output_dir_rel);

  beam_mapping::Poses bag_poses;
  std::string bag_poses_file_path = "/home/nick/bag_files/ig_scans/2019_02_13_Structures_Lab/ig_scan_2019-02-13-19-44-24-loam-short1.bag";
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
