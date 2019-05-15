#define CATCH_CONFIG_MAIN
#include "beam_calibration/Pinhole.h"
#include "beam_calibration/TfTree.h"
#include "beam_colorize/Projection.h"
#include "beam_utils/math.hpp"

#include <boost/filesystem.hpp>

#include <iostream>

#include <catch2/catch.hpp>
#include <nlohmann/json.hpp>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PerformColorization(int type) {
  // load intrinsics
  std::string intrinsics_name = "F1.json";
  std::string intrinsics_location = __FILE__;
  intrinsics_location.erase(intrinsics_location.end() - 17,
                            intrinsics_location.end());
  intrinsics_location += "test_data/" + intrinsics_name;
  std::shared_ptr<beam_calibration::Intrinsics> F1 =
      std::make_shared<beam_calibration::Pinhole>();
  F1->LoadJSON(intrinsics_location);

  // load Image
  std::string image_name = "test_img.jpg";
  std::string image_location = __FILE__;
  image_location.erase(image_location.end() - 17, image_location.end());
  image_location += "test_data/" + image_name;
  INFO(image_location);
  cv::Mat image;
  image = cv::imread(image_location, CV_LOAD_IMAGE_COLOR);
  if (!image.data) {
    std::cout << "Could not open or find the image" << std::endl;
  } else {
    LOG_INFO("Opened file: %s", image_location.c_str());
  }

  // load pcd
  std::string pcd_name = "map18crop.pcd";
  std::string pcd_location = __FILE__;
  pcd_location.erase(pcd_location.end() - 17, pcd_location.end());
  pcd_location += "test_data/" + pcd_name;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_location, *cloud) == -1) {
    LOG_INFO("Couldn't read pcd file:  %s\n", pcd_location.c_str());
  } else {
    LOG_INFO("Opened file: %s", pcd_location.c_str());
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  beam_colorize::ColorizerType type2;
  if (type == 0) {
    type2 = beam_colorize::ColorizerType::PROJECTION;
  } else {
    type2 = beam_colorize::ColorizerType::RAY_TRACE;
  }
  auto colorizer = beam_colorize::Colorizer::Create(type2);
  bool image_distorted = true;
  colorizer->SetPointCloud(cloud);
  colorizer->SetImage(image);
  colorizer->SetIntrinsics(F1.get());
  colorizer->SetDistortion(image_distorted);
  cloud_colored = colorizer->ColorizePointCloud();

  return cloud_colored;
}

TEST_CASE("Test correct projection colorization") {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored = PerformColorization(0);

  nlohmann::json J;
  std::string data_location = __FILE__;
  data_location.erase(data_location.end() - 17, data_location.end());
  data_location += "/test_data/test_points.json";
  std::ifstream file(data_location);
  file >> J;

  auto red_points = J["projection"]["red_points"];
  auto blue_points = J["projection"]["blue_points"];

  // compute number of points that are correctly colored red
  uint32_t correct_red = 0;
  for (uint32_t i = 0; i < red_points.size(); i++) {
    int idx = red_points[i];
    int r = (int)cloud_colored->points[idx].r,
        g = (int)cloud_colored->points[idx].g,
        b = (int)cloud_colored->points[idx].b;
    if (r == 254 && g == 0 && b == 0) { correct_red++; }
  }
  float percent_red = (float)correct_red / (float)red_points.size();
  // compute number of points correctly colored blue
  uint32_t correct_blue = 0;
  for (uint32_t i = 0; i < blue_points.size(); i++) {
    int idx = blue_points[i];
    int r = (int)cloud_colored->points[idx].r,
        g = (int)cloud_colored->points[idx].g,
        b = (int)cloud_colored->points[idx].b;
    if (r == 0 && g == 0 && b == 254) { correct_blue++; }
  }
  float percent_blue = (float)correct_blue / (float)blue_points.size();

  REQUIRE(percent_red > 0.95);
  REQUIRE(percent_blue > 0.95);
}

TEST_CASE("Test correct raytrace colorization") {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored = PerformColorization(1);

  nlohmann::json J;
  std::string data_location = __FILE__;
  data_location.erase(data_location.end() - 17, data_location.end());
  data_location += "/test_data/test_points.json";
  std::ifstream file(data_location);
  file >> J;

  auto red_points = J["raytrace"]["red_points"];
  auto blue_points = J["raytrace"]["blue_points"];

  // compute number of points that are correctly colored red
  uint32_t correct_red = 0;
  for (uint32_t i = 0; i < red_points.size(); i++) {
    int idx = red_points[i];
    int r = (int)cloud_colored->points[idx].r,
        g = (int)cloud_colored->points[idx].g,
        b = (int)cloud_colored->points[idx].b;
    if (r == 254 && g == 0 && b == 0) { correct_red++; }
  }
  float percent_red = (float)correct_red / (float)red_points.size();
  // compute number of points correctly colored blue
  uint32_t correct_blue = 0;
  for (uint32_t i = 0; i < blue_points.size(); i++) {
    int idx = blue_points[i];
    int r = (int)cloud_colored->points[idx].r,
        g = (int)cloud_colored->points[idx].g,
        b = (int)cloud_colored->points[idx].b;
    if (r == 0 && g == 0 && b == 254) { correct_blue++; }
  }
  float percent_blue = (float)correct_blue / (float)blue_points.size();

  REQUIRE(percent_red > 0.95);
  REQUIRE(percent_blue > 0.95);
}