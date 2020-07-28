#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <beam_calibration/Radtan.h>
#include <beam_cv/DepthMap.h>
#include <beam_cv/Utils.h>

TEST_CASE("Test Constructor, Getters/Setters.") {
  // file locations
  std::string cur_location = __FILE__;
  cur_location.erase(cur_location.end() - 23, cur_location.end());
  cur_location += "tests/test_data/";
  std::string intrinsics_loc = cur_location + "F1.json";
  // load other objects
  cv::Mat img = cv::imread(cur_location + "test.jpg", cv::IMREAD_COLOR);
  std::shared_ptr<beam_calibration::CameraModel> F1 =
      std::make_shared<beam_calibration::Radtan>(intrinsics_loc);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>(cur_location + "test.pcd", *cloud);
  // test method exception throwing
  beam_cv::DepthMap dm;
  REQUIRE_THROWS(dm.ExtractDepthMap(0.03, 5));
  REQUIRE_THROWS(dm.ExtractPointCloud());
  REQUIRE_NOTHROW(dm.SetCloud(cloud));
  REQUIRE_NOTHROW(dm.SetCameraModel(F1));
  REQUIRE_NOTHROW(dm.SetDepthImage(img));
}

TEST_CASE("Test Depth map extractor.") {
  // file locations
  std::string cur_location = __FILE__;
  cur_location.erase(cur_location.end() - 23, cur_location.end());
  cur_location += "tests/test_data/";
  std::string intrinsics_loc = cur_location + "F2.json";
  // load other objects
  std::shared_ptr<beam_calibration::CameraModel> F1 =
      std::make_shared<beam_calibration::Radtan>(intrinsics_loc);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>(cur_location + "259_map.pcd", *cloud);
  // test method exception throwing
  beam_cv::DepthMap dm(F1, cloud);
  dm.ExtractDepthMap(0.1, 1);
  int num_out = 0;
  cv::Mat depth = dm.GetDepthImage();
  cv::Mat img = cv::imread(cur_location + "259_mask.jpg", cv::IMREAD_GRAYSCALE);
  for (int row = 0; row < depth.rows; row++) {
    for (int col = 0; col < depth.cols; col++) {
      float distance = depth.at<float>(row, col);
      if (distance > 0.001 && img.at<uint8_t>(row, col) == 0) { num_out++; }
    }
  }
  REQUIRE(num_out < 200);
}
