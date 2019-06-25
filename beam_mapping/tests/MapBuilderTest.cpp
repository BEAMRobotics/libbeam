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

TEST_CASE("Testing map building") {
  std::string current_file_rel, config_file_rel, poses_file_rel, bag_file_rel,
              bag_file_path, poses_file_path, config_file_path;
  current_file_rel = "/tests/MapBuilderTest.cpp";
  config_file_rel = "/tests/test_data/MapBuilderTestConfig.json";
  poses_file_rel = "/tests/test_data/PosesTest.json";
  bag_file_rel = "/tests/test_data/BagTest.bag";

  config_file_path = GetFullFile(current_file_rel, config_file_rel);
  poses_file_path = GetFullFile(current_file_rel, poses_file_rel);

  // TODO: Change this back
  // bag_file_path = GetFullFile(current_file_rel, bag_file_rel);
  bag_file_path = "/home/nick/bag_files/ig_scans/2019_02_13_Structures_Lab/ig_scan_2019-02-13-19-44-24.bag";

  beam_mapping::MapBuilder map_builder(config_file_path);
  map_builder.OverrideBagFile(bag_file_path);
  map_builder.OverridePosesFile(poses_file_path);
  map_builder.BuildMap();
  REQUIRE(1 == 1);
}
