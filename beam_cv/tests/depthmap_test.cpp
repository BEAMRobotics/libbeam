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
  REQUIRE_THROWS(dm.DepthInterpolation(70, 5, 0.05, 1));
  REQUIRE_THROWS(dm.ExtractDepthMap(0.03, 5));
  REQUIRE_THROWS(dm.KMeansCompletion(0.03, img));
  REQUIRE_THROWS(dm.ExtractPointCloud());
  REQUIRE_NOTHROW(dm.SetCloud(cloud));
  REQUIRE_NOTHROW(dm.SetModel(F1));
  REQUIRE_THROWS(dm.SetDepthImage(img));
}

TEST_CASE("Test Depth map extractor.") {
  // file locations
  std::string cur_location = __FILE__;
  cur_location.erase(cur_location.end() - 23, cur_location.end());
  cur_location += "tests/test_data/";
  std::string intrinsics_loc = cur_location + "F1.json";

  // load other objects
  std::shared_ptr<beam_calibration::CameraModel> F1 =
      std::make_shared<beam_calibration::Radtan>(intrinsics_loc);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>(cur_location + "test.pcd", *cloud);

  // test method exception throwing
  beam_cv::DepthMap dm(F1, cloud);
  int low_density = dm.ExtractDepthMap(0.02, 3);
  int high_density = dm.ExtractDepthMap(0.1, 20);
  REQUIRE(low_density < high_density);
}

TEST_CASE("Test Depth interpolation.") {
  // file locations
  std::string cur_location = __FILE__;
  cur_location.erase(cur_location.end() - 23, cur_location.end());
  cur_location += "tests/test_data/";
  std::string intrinsics_loc = cur_location + "F1.json";

  // load other objects
  std::shared_ptr<beam_calibration::CameraModel> F1 =
      std::make_shared<beam_calibration::Radtan>(intrinsics_loc);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>(cur_location + "test.pcd", *cloud);

  // test method exception throwing
  beam_cv::DepthMap dm(F1, cloud);
  dm.ExtractDepthMap(0.03, 3);
  REQUIRE(dm.DepthInterpolation(70, 5, 0.05, 1) > 100000);
}
