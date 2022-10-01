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
#include <beam_colorize/ProjectionOcclusionSafe.h>
#include <beam_containers/ImageBridge.h>
#include <beam_mapping/Poses.h>
#include <beam_utils/pointclouds.h>

std::string GetTestFileRoot() {
  std::string current_file_path = "projection_occlusion_safe_test.cpp";
  std::string cur_dir = __FILE__;
  cur_dir.erase(cur_dir.end() - current_file_path.length(), cur_dir.end());
  return beam::CombinePaths(
      std::vector<std::string>{cur_dir, "test_data", "parking_garage_dataset"});
}

std::string root_path_ = GetTestFileRoot();
std::string poses_path_ = beam::CombinePaths(root_path_, "poses.json");
std::string extrinsics_path_ = beam::CombinePaths(std::vector<std::string>{
    root_path_, "calibration", "extrinsics", "extrinsics.json"});
std::string intrinsics_path_ = beam::CombinePaths(std::vector<std::string>{
    root_path_, "calibration", "intrinsics", "F1.json"});
std::string image_container_path_ = beam::CombinePaths(
    std::vector<std::string>{root_path_, "images", "ImageBridge1"});
std::string tmp_path_ = boost::filesystem::temp_directory_path().string();

void SaveMap(const PointCloudCol& map, const std::string& filename) {
  std::string output_file = beam::CombinePaths(tmp_path_, filename);
  std::string error;
  if (!beam::SavePointCloud<PointTypeCol>(
          output_file, map, beam::PointCloudFileType::PCDBINARY, error)) {
    std::cout << "unable to save results to: " << output_file << "\n";
    std::cout << "error: " << error << "\n";
  } else {
    std::cout << "saved results to: " << output_file << "\n";
  }
}

void SaveMap(const pcl::PointCloud<beam_containers::PointBridge>& map,
             const std::string& filename) {
  std::string output_file = beam::CombinePaths(tmp_path_, filename);
  std::string error;
  if (!beam::SavePointCloud<beam_containers::PointBridge>(
          output_file, map, beam::PointCloudFileType::PCDBINARY, error)) {
    std::cout << "unable to save results to: " << output_file << "\n";
    std::cout << "error: " << error << "\n";
  } else {
    std::cout << "saved results to: " << output_file << "\n";
  }
}

// to be validate by viewing resulting maps
TEST_CASE("Test on full datasets") {
  // get file paths
  std::string map_path =
      beam::CombinePaths(std::vector<std::string>{root_path_, "map.pcd"});
  // load data
  std::shared_ptr<beam_calibration::CameraModel> camera_model =
      beam_calibration::CameraModel::Create(intrinsics_path_);

  PointCloud map;
  pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, map);

  beam_calibration::TfTree extinsics_tree;
  extinsics_tree.LoadJSON(extrinsics_path_);

  beam_calibration::TfTree poses_tree;
  beam_mapping::Poses poses_container;
  poses_container.LoadFromJSON(poses_path_);
  const auto& poses = poses_container.GetPoses();
  const auto& timestamps = poses_container.GetTimeStamps();
  for (uint8_t i = 0; i < poses.size(); i++) {
    poses_tree.AddTransform(Eigen::Affine3d(poses[i]),
                            poses_container.GetFixedFrame(),
                            poses_container.GetMovingFrame(), timestamps[i]);
  }

  beam_containers::ImageBridge image_container;
  image_container.LoadFromJSON(image_container_path_);

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

  // create colorizer
  auto colorizer = beam_colorize::Colorizer::Create(
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
  SaveMap(*map_colored, "projection_occlusion_full_regproj.pcd");

  // ---------------------------------------------------------
  // Colorize with occlusion safe projection

  // create colorizer
  beam_colorize::ProjectionOcclusionSafe colorizer2;
  colorizer2.SetPointCloud(map_in_cam_frame);
  colorizer2.SetImage(image_container.GetBGRImage());
  colorizer2.SetDistortion(image_container.GetBGRIsDistorted());
  colorizer2.SetIntrinsics(camera_model);
  //   colorizer2.SetWindowSize(10);
  //   colorizer2.SetWindowStride(4);
  //   colorizer2.SetDepthThreshold(0.3);
  // these were picked from the params search's best result from the commented
  // out code below
  colorizer2.SetWindowSize(90);
  colorizer2.SetWindowStride(67);
  colorizer2.SetDepthThreshold(0.2);

  // colorize map
  map_colored = std::make_shared<PointCloudCol>();
  map_colored_in_cam = colorizer2.ColorizePointCloud();
  pcl::transformPointCloud(*map_colored_in_cam, *map_colored, T_MAP_CAM);
  *map_colored = beam::AddFrameToCloud(*map_colored, T_MAP_CAM);
  SaveMap(*map_colored, "projection_occlusion_full_safeproj.pcd");
}

TEST_CASE("column occlusion") {
  // get map path
  std::string map_path = beam::CombinePaths(
      std::vector<std::string>{root_path_, "column_occlusion.pcd"});

  // load data
  std::shared_ptr<beam_calibration::CameraModel> camera_model =
      beam_calibration::CameraModel::Create(intrinsics_path_);

  PointCloud map;
  pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, map);

  beam_calibration::TfTree extinsics_tree;
  extinsics_tree.LoadJSON(extrinsics_path_);

  beam_calibration::TfTree poses_tree;
  beam_mapping::Poses poses_container;
  poses_container.LoadFromJSON(poses_path_);
  const auto& poses = poses_container.GetPoses();
  const auto& timestamps = poses_container.GetTimeStamps();
  for (uint8_t i = 0; i < poses.size(); i++) {
    poses_tree.AddTransform(Eigen::Affine3d(poses[i]),
                            poses_container.GetFixedFrame(),
                            poses_container.GetMovingFrame(), timestamps[i]);
  }

  beam_containers::ImageBridge image_container;
  image_container.LoadFromJSON(image_container_path_);

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

  // create colorizer
  beam_colorize::ProjectionOcclusionSafe colorizer;
  colorizer.SetPointCloud(map_in_cam_frame);
  colorizer.SetImage(image_container.GetBGRImage());
  colorizer.SetDistortion(image_container.GetBGRIsDistorted());
  colorizer.SetIntrinsics(camera_model);
  // these were picked from the params search's best result from the commented
  // out code below
  colorizer.SetWindowSize(90);
  colorizer.SetWindowStride(67);
  colorizer.SetDepthThreshold(0.2);

  // colorize map
  auto map_colored = std::make_shared<PointCloudCol>();
  auto map_colored_in_cam = colorizer.ColorizePointCloud();
  pcl::transformPointCloud(*map_colored_in_cam, *map_colored, T_MAP_CAM);
  *map_colored = beam::AddFrameToCloud(*map_colored, T_MAP_CAM);
  SaveMap(*map_colored, "column_occlusion_test.pcd");

  // test against expected
  int expected_num_colored{30041};
  int expected_num_occluded{24199};
  int count_colored{0};
  int count_occluded{0};
  for (pcl::PointXYZRGB p : *map_colored_in_cam) {
    if (p.r + p.g + p.b == 0) {
      count_occluded++;
    } else {
      count_colored++;
    }
  }

  std::cout << "from a total of " << map_colored->size() << " points, "
            << count_colored << " were colored and " << count_occluded
            << " were occluded\n";
  REQUIRE(expected_num_colored + expected_num_occluded == map.size());
  REQUIRE(count_occluded + count_colored == map.size());
  double tol{0.4};
  REQUIRE(std::abs(count_occluded - expected_num_occluded) <
          tol * expected_num_occluded);
  REQUIRE(std::abs(count_colored - expected_num_colored) <
          tol * expected_num_colored);
}

// to be validate by viewing resulting maps
TEST_CASE("Test mask colorization") {
  // get file paths
  std::string map_path =
      beam::CombinePaths(std::vector<std::string>{root_path_, "map.pcd"});
  // load data
  std::shared_ptr<beam_calibration::CameraModel> camera_model =
      beam_calibration::CameraModel::Create(intrinsics_path_);

  PointCloud map;
  pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, map);

  beam_calibration::TfTree extinsics_tree;
  extinsics_tree.LoadJSON(extrinsics_path_);

  beam_calibration::TfTree poses_tree;
  beam_mapping::Poses poses_container;
  poses_container.LoadFromJSON(poses_path_);
  const auto& poses = poses_container.GetPoses();
  const auto& timestamps = poses_container.GetTimeStamps();
  for (uint8_t i = 0; i < poses.size(); i++) {
    poses_tree.AddTransform(Eigen::Affine3d(poses[i]),
                            poses_container.GetFixedFrame(),
                            poses_container.GetMovingFrame(), timestamps[i]);
  }

  beam_containers::ImageBridge image_container;
  image_container.LoadFromJSON(image_container_path_);

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

  // create colorizer
  beam_colorize::ProjectionOcclusionSafe colorizer;
  colorizer.SetPointCloud(map_in_cam_frame);
  colorizer.SetImage(image_container.GetBGRMask());
  colorizer.SetDistortion(image_container.GetBGRIsDistorted());
  colorizer.SetIntrinsics(camera_model);
  //   colorizer.SetWindowSize(10);
  //   colorizer.SetWindowStride(4);
  //   colorizer.SetDepthThreshold(0.3);
  // these were picked from the params search's best result from the commented
  // out code below
  colorizer.SetWindowSize(90);
  colorizer.SetWindowStride(67);
  colorizer.SetDepthThreshold(0.2);

  // colorize map
  pcl::PointCloud<beam_containers::PointBridge>::Ptr defect_map =
      colorizer.ColorizeMask();
  pcl::PointCloud<beam_containers::PointBridge> defect_map_in_map_frame;
  pcl::transformPointCloud(*defect_map, defect_map_in_map_frame, T_MAP_CAM);
  SaveMap(defect_map_in_map_frame, "projection_occlusion_safe_mask.pcd");
}

/*
TEST_CASE("param searching") {
  // get map path
  std::string map_path = beam::CombinePaths(
      std::vector<std::string>{root_path_, "various_occlusion_testing.pcd"});

  // load data
  std::shared_ptr<beam_calibration::CameraModel> camera_model =
      beam_calibration::CameraModel::Create(intrinsics_path_);

  PointCloud map;
  pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, map);

  beam_calibration::TfTree extinsics_tree;
  extinsics_tree.LoadJSON(extrinsics_path_);

  beam_calibration::TfTree poses_tree;
  beam_mapping::Poses poses_container;
  poses_container.LoadFromJSON(poses_path_);
  const auto& poses = poses_container.GetPoses();
  const auto& timestamps = poses_container.GetTimeStamps();
  for (uint8_t i = 0; i < poses.size(); i++) {
    poses_tree.AddTransform(Eigen::Affine3d(poses[i]),
                            poses_container.GetFixedFrame(),
                            poses_container.GetMovingFrame(), timestamps[i]);
  }

  beam_containers::ImageBridge image_container;
  image_container.LoadFromJSON(image_container_path_);

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

  // create colorizer
  beam_colorize::ProjectionOcclusionSafe colorizer;
  colorizer.SetPointCloud(map_in_cam_frame);
  colorizer.SetImage(image_container.GetBGRImage());
  colorizer.SetDistortion(image_container.GetBGRIsDistorted());
  colorizer.SetIntrinsics(camera_model);

  int expected_num_colored{93796};
  int expected_num_occluded{29274};

  // param search:
  nlohmann::json J;
  nlohmann::json J_best;
  nlohmann::json J_worst;
  int count = 0;
  struct timespec t;
  for (int window_size = 5; window_size < 150; window_size += 5) {
    std::vector<int> strides{window_size * 0.25, window_size * 0.5,
                             window_size * 0.75};
    for (int stride : strides) {
      for (double depth_thres = 0.1; depth_thres < 0.3; depth_thres += 0.1) {
        count++;
        colorizer.SetWindowSize(window_size);
        colorizer.SetWindowStride(stride);
        colorizer.SetDepthThreshold(depth_thres);

        // colorize map
        auto map_colored = std::make_shared<PointCloudCol>();
        beam::tic(&t);
        auto map_colored_in_cam = colorizer.ColorizePointCloud();
        float time_elapsed = beam::toc(&t);
        pcl::transformPointCloud(*map_colored_in_cam, *map_colored, T_MAP_CAM);
        *map_colored = beam::AddFrameToCloud(*map_colored, T_MAP_CAM);
        std::string output_file = "test" + std::to_string(count) + ".pcd";
        SaveMap(*map_colored, output_file);
        int count_colored{0};
        int count_occluded{0};
        for (pcl::PointXYZRGB p : *map_colored_in_cam) {
          if (p.r + p.g + p.b == 0) {
            count_occluded++;
          } else {
            count_colored++;
          }
        }
        nlohmann::json J2;
        J2["output"] = output_file;
        J2["window_size"] = window_size;
        J2["stride"] = stride;
        J2["depth_thres"] = depth_thres;
        J2["col_count"] = count_colored;
        J2["ocl_count"] = count_occluded;
        double accuracy = static_cast<double>(
                              std::abs(expected_num_colored - count_colored)) /
                          static_cast<double>(expected_num_colored);
        J2["accuracy"] = accuracy;
        J2["time_elapsed"] = time_elapsed;
        if (count == 1) {
          J_best = J2;
          J_worst = J2;
        } else if (accuracy < J_worst["accuracy"]) {
          J_worst = J2;
        } else if (accuracy > J_best["accuracy"]) {
          J_best = J2;
        }
        J.push_back(J2);
      }
    }
  }

  // save jsons
  std::string json_path =
      beam::CombinePaths(tmp_path_, "projection_occlusion_safe_summary.json");
  std::cout << "saving to: " << json_path << "\n";
  std::ofstream o(json_path);
  o << std::setw(4) << J << std::endl;
  json_path =
      beam::CombinePaths(tmp_path_, "projection_occlusion_safe_best.json");
  std::cout << "saving to: " << json_path << "\n";
  std::ofstream o2(json_path);
  o2 << std::setw(4) << J_best << std::endl;
  json_path =
      beam::CombinePaths(tmp_path_, "projection_occlusion_safe_worst.json");
  std::cout << "saving to: " << json_path << "\n";
  std::ofstream o3(json_path);
  o3 << std::setw(4) << J_worst << std::endl;
}
*/