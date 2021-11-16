#define CATCH_CONFIG_MAIN

#include <boost/filesystem.hpp>
#include <catch2/catch.hpp>

#include <beam_mapping/MapBuilder.h>
#include <beam_utils/math.h>

std::string data_path_ =
    beam::LibbeamRoot() + "beam_mapping/tests/test_data/MapBuilderTests/";

TEST_CASE("Testing map building with JSON") {
  std::string config_file_path = data_path_ + "TestConfigJSON.json";
  std::string pose_file_path = data_path_ + "TestPosesJSON.json";
  std::string bag_file_path = data_path_ + "TestBagJSON.bag";
  std::string extrinsics_file_path = data_path_ + "extrinsics.json";
  std::string output_dir_path = data_path_ + "tmp/";
  boost::filesystem::create_directory(output_dir_path);
  beam_mapping::MapBuilder map_builder(config_file_path, pose_file_path,
                                       output_dir_path, extrinsics_file_path);
  REQUIRE_NOTHROW(map_builder.BuildMap());
  boost::filesystem::remove_all(output_dir_path);
}

TEST_CASE("Testing map building with PLY") {
  std::string config_file_path = data_path_ + "TestConfigPLY.json";
  std::string pose_file_path = data_path_ + "TestPosesPLY.ply";
  std::string bag_file_path = data_path_ + "TestBagPLY.bag";
  std::string extrinsics_file_path = data_path_ + "extrinsics.json";
  std::string output_dir_path = data_path_ + "tmp/";
  std::string moving_frame = "vvlp_link";
  
  boost::filesystem::create_directory(output_dir_path);
  beam_mapping::MapBuilder map_builder(config_file_path, pose_file_path,
                                       output_dir_path, extrinsics_file_path,
                                       moving_frame);
  REQUIRE_NOTHROW(map_builder.BuildMap());
  boost::filesystem::remove_all(output_dir_path);
}

TEST_CASE("Testing map building with PCD") {
  std::string config_file_path = data_path_ + "TestConfigPCD.json";
  std::string pose_file_path = data_path_ + "TestPosesPCD.pcd";
  std::string bag_file_path = data_path_ + "TestBagPCD.bag";
  std::string extrinsics_file_path = data_path_ + "extrinsics.json";
  std::string output_dir_path = data_path_ + "tmp/";

  std::string moving_frame = "base_link";
  boost::filesystem::create_directory(output_dir_path);
  beam_mapping::MapBuilder map_builder(config_file_path, pose_file_path,
                                       output_dir_path, extrinsics_file_path,
                                       moving_frame);
  REQUIRE_NOTHROW(map_builder.BuildMap());
  boost::filesystem::remove_all(output_dir_path);
}

/**
 * tmp test:
TEST_CASE("Testing map building from Bag odometry") {
  std::string config_file_path = data_path_ + "TestConfigBAG.json";
  std::string bag_file_path = data_path_ + "TestBagBAG.bag";
  std::string extrinsics_file_path = data_path_ + "extrinsics.json";
  std::string output_dir_path = data_path_ + "tmp/";

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
*/
