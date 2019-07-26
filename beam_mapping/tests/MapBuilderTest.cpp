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

//TODO: add test bag and pose files then remove generated maps 
TEST_CASE("Testing map building with JSON") {
  std::string current_file_rel, config_file_rel, pose_file_rel, bag_file_rel,
              bag_file_path, pose_file_path, config_file_path;
  current_file_rel = "/tests/MapBuilderTest.cpp";
  config_file_rel = "/tests/test_data/MapBuilderTestConfig.json";
  pose_file_rel = "/tests/test_data/PosesTest.json";
  bag_file_rel = "/tests/test_data/BagTest.bag";

  config_file_path = GetFullFile(current_file_rel, config_file_rel);
  pose_file_path = GetFullFile(current_file_rel, pose_file_rel);
  bag_file_path = GetFullFile(current_file_rel, bag_file_rel);

  // TODO: Change this back to test files
  pose_file_path = "/home/nick/results/ig_maps/libbeam_map_builder/testing/2019_7_8_17_43_5_poses.json";
  bag_file_path = "/home/nick/results/ig_maps/libbeam_map_builder/testing/ig_scan_2019-02-13-19-44-24-loam-short1.bag";

  beam_mapping::MapBuilder map_builder(config_file_path);
  map_builder.OverrideBagFile(bag_file_path);
  map_builder.OverridePoseFile(pose_file_path);
  map_builder.BuildMap();
  REQUIRE(1 == 1);
}

//TODO: add test bag and pose files then remove generated maps
/*
TEST_CASE("Testing map building with PLY") {
  std::string current_file_rel, config_file_rel, pose_file_rel, bag_file_rel,
              bag_file_path, pose_file_path, config_file_path;
  current_file_rel = "/tests/MapBuilderTest.cpp";
  config_file_rel = "/tests/test_data/MapBuilderTestConfigKaarta.json";
  pose_file_rel = "/tests/test_data/PosesTestKaarta.ply";
  bag_file_rel = "/tests/test_data/BagTestKaarta.bag";

  config_file_path = GetFullFile(current_file_rel, config_file_rel);
  pose_file_path = GetFullFile(current_file_rel, pose_file_rel);
  bag_file_path = GetFullFile(current_file_rel, bag_file_rel);

  // TODO: Change this back to test files
  pose_file_path = "/home/nick/results/ig_maps/libbeam_map_builder/testing/trajectory_2019-07-22-23-00-44_loop_closed.ply";
  bag_file_path = "/home/nick/results/ig_maps/libbeam_map_builder/testing/ig_scan_2019-07-22-23-00-36-kaarta.bag";

  beam_mapping::MapBuilder map_builder(config_file_path);
  map_builder.OverrideBagFile(bag_file_path);
  map_builder.OverridePoseFile(pose_file_path);
  std::string moving_frame = "hvlp_link";
  std::string fixed_frame = "odom";
  map_builder.SetPosesMovingFrame(moving_frame);
  map_builder.SetPosesFixedFrame(fixed_frame);
  map_builder.BuildMap();
  REQUIRE(1 == 1);
}
*/
