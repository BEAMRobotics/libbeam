#define CATCH_CONFIG_MAIN

#include <iostream>
#include <typeinfo>

#include <boost/filesystem.hpp>
#include <catch2/catch.hpp>
#include <nlohmann/json.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <beam_calibration/CameraModel.h>
#include <beam_calibration/TfTree.h>
#include <beam_colorize/Projection.h>
#include <beam_containers/ImageBridge.h>
#include <beam_mapping/Poses.h>
#include <beam_utils/pointclouds.h>
// #include <beam_colorize/ProjectionOcclusionSafe.h>

std::string GetTestFileRoot() {
  std::string current_file_path = "projection_occlusion_safe_test.cpp";
  std::string cur_dir = __FILE__;
  cur_dir.erase(cur_dir.end() - current_file_path.length(), cur_dir.end());
  return beam::CombinePaths(
      std::vector<std::string>{cur_dir, "test_data", "parking_garage_dataset"});
}

TEST_CASE("Test correct projection colorization") {
  // get file paths
  std::string root_path = GetTestFileRoot();
  std::string map_path =
      beam::CombinePaths(std::vector<std::string>{root_path, "map.pcd"});
  std::string poses_path =
      beam::CombinePaths(std::vector<std::string>{root_path, "poses.json"});
  std::string extrinsics_path = beam::CombinePaths(std::vector<std::string>{
      root_path, "calibration", "extrinsics", "extrinsics.json"});
  std::string intrinsics_path = beam::CombinePaths(std::vector<std::string>{
      root_path, "calibration", "intrinsics", "F1.json"});
  std::string image_container_path = beam::CombinePaths(
      std::vector<std::string>{root_path, "images", "ImageBridge1"});

  // load data
  std::shared_ptr<beam_calibration::CameraModel> camera_model =
      beam_calibration::CameraModel::Create(intrinsics_path);

  PointCloud map;
  pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, map);

  beam_calibration::TfTree extinsics_tree;
  extinsics_tree.LoadJSON(extrinsics_path);

  beam_calibration::TfTree poses_tree;
  beam_mapping::Poses poses_container;
  poses_container.LoadFromJSON(poses_path);
  const auto& poses = poses_container.GetPoses();
  const auto& timestamps = poses_container.GetTimeStamps();
  for (uint8_t i = 0; i < poses.size(); i++) {
    poses_tree.AddTransform(Eigen::Affine3d(poses[i]),
                            poses_container.GetFixedFrame(),
                            poses_container.GetMovingFrame(), timestamps[i]);
  }

  beam_containers::ImageBridge image_container;
  image_container.LoadFromJSON(image_container_path);

  // transform map into camera frame
  Eigen::Matrix4d T_MAP_MOVING =
      poses_tree
          .GetTransformEigen(poses_container.GetFixedFrame(),
                             poses_container.GetMovingFrame(),
                             image_container.GetRosTime())
          .matrix();

  Eigen::Matrix4d T_MOVING_CAM =
      extinsics_tree
          .GetTransformEigen(poses_container.GetMovingFrame(),
                             image_container.GetBGRFrameId())
          .matrix();

  Eigen::Matrix4d T_MAP_CAM = T_MAP_MOVING * T_MOVING_CAM;

  PointCloud::Ptr map_in_cam_frame = std::make_shared<PointCloud>();
  pcl::transformPointCloud(map, *map_in_cam_frame,
                           beam::InvertTransform(T_MAP_CAM));

  // ---------------------------------------------------------
  // Colorize with regular projection

  // create colorizer
  std::unique_ptr<beam_colorize::Colorizer> colorizer =
      beam_colorize::Colorizer::Create(
          beam_colorize::ColorizerType::PROJECTION);
  colorizer->SetPointCloud(map_in_cam_frame);
  colorizer->SetImage(image_container.GetBGRImage());
  colorizer->SetDistortion(image_container.GetBGRIsDistorted());
  colorizer->SetIntrinsics(camera_model);

  // colorize map
  PointCloudColPtr map_colored_in_cam = colorizer->ColorizePointCloud();
  PointCloudColPtr map_colored = std::make_shared<PointCloudCol>();
  pcl::transformPointCloud(*map_colored_in_cam, *map_colored, T_MAP_CAM);
  *map_colored = beam::AddFrameToCloud(*map_colored, T_MAP_CAM);

  // save
  std::string tmp_path = boost::filesystem::temp_directory_path().string();
  std::string output_file = beam::CombinePaths(
      tmp_path, "projection_occlusion_safe_test_result1.pcd");
  std::string error;
  if (!beam::SavePointCloud<PointTypeCol>(output_file, *map_colored,
                                          beam::PointCloudFileType::PCDBINARY,
                                          error)) {
    std::cout << "unable to save results to: " << output_file << "\n";
    std::cout << "error: " << error << "\n";
  } else {
    std::cout << "saved results to: " << output_file << "\n";
  }

  // ---------------------------------------------------------
  // Colorize with occlusion safe projection

  // create colorizer
  colorizer = beam_colorize::Colorizer::Create(
      beam_colorize::ColorizerType::PROJECTION_OCCLUSION_SAFE);
  colorizer->SetPointCloud(map_in_cam_frame);
  colorizer->SetImage(image_container.GetBGRImage());
  colorizer->SetDistortion(image_container.GetBGRIsDistorted());
  colorizer->SetIntrinsics(camera_model);

  // colorize map
  map_colored_in_cam = colorizer->ColorizePointCloud();
  map_colored = std::make_shared<PointCloudCol>();
  pcl::transformPointCloud(*map_colored_in_cam, *map_colored, T_MAP_CAM);
  *map_colored = beam::AddFrameToCloud(*map_colored, T_MAP_CAM);

  // save
  output_file = beam::CombinePaths(
      tmp_path, "projection_occlusion_safe_test_result2.pcd");
  if (!beam::SavePointCloud<PointTypeCol>(output_file, *map_colored,
                                          beam::PointCloudFileType::PCDBINARY,
                                          error)) {
    std::cout << "unable to save results to: " << output_file << "\n";
    std::cout << "error: " << error << "\n";
  } else {
    std::cout << "saved results to: " << output_file << "\n";
  }
}
