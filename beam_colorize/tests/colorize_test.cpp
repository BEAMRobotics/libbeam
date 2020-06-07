#define CATCH_CONFIG_MAIN

#include <iostream>
#include <typeinfo>

#include <boost/filesystem.hpp>
#include <catch2/catch.hpp>
#include <nlohmann/json.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <beam_calibration/Radtan.h>
#include <beam_calibration/TfTree.h>
#include <beam_colorize/Projection.h>
#include <beam_colorize/RayTrace.h>
#include <beam_utils/math.hpp>

pcl::PointCloud<pcl::PointXYZ>::Ptr GetPCD(std::string name) {
  std::string cur_dir = __FILE__;
  cur_dir.erase(cur_dir.end() - 17, cur_dir.end());
  cur_dir += "test_data/";
  std::string pcd_location = cur_dir + name;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_location, *cloud) == -1) {
    BEAM_INFO("Couldn't read pcd file:  {}\n", pcd_location);
  } else {
    BEAM_INFO("Opened file: {}", pcd_location);
  }
  return cloud;
}

cv::Mat GetImage(std::string name) {
  std::string cur_dir = __FILE__;
  cur_dir.erase(cur_dir.end() - 17, cur_dir.end());
  cur_dir += "test_data/";
  std::string image_location = cur_dir + name;
  cv::Mat image;
  image = cv::imread(image_location, CV_LOAD_IMAGE_COLOR);
  if (!image.data) {
    std::cout << "Could not open or find the image" << std::endl;
  } else {
    BEAM_INFO("Opened file: {}", image_location);
  }
  return image;
}

std::shared_ptr<beam_calibration::CameraModel> GetIntrinsics(std::string name) {
  std::string cur_dir = __FILE__;
  cur_dir.erase(cur_dir.end() - 17, cur_dir.end());
  cur_dir += "test_data/";
  std::string intrinsics_location = cur_dir + name;
  std::shared_ptr<beam_calibration::CameraModel> model =
      std::make_shared<beam_calibration::Radtan>(intrinsics_location);
  return model;
}

TEST_CASE("Test correct projection colorization") {
  auto colorizer = beam_colorize::Colorizer::Create(
      beam_colorize::ColorizerType::PROJECTION);

  // load intrinsics
  std::string intrinsics_name = "camera0.json";
  std::string test1_pcd = "test1/101_map.pcd";
  std::string test2_pcd = "test2/259_map.pcd";
  std::string test1_img = "test1/101_mask.jpg";
  std::string test2_img = "test2/259_mask.jpg";
  std::shared_ptr<beam_calibration::CameraModel> model =
      GetIntrinsics(intrinsics_name);
  bool image_distorted = false;
  // Test case 1
  cv::Mat image = GetImage(test1_img);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = GetPCD(test1_pcd);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  colorizer->SetImage(image);
  colorizer->SetIntrinsics(model);
  colorizer->SetDistortion(image_distorted);
  colorizer->SetPointCloud(cloud);
  cloud_colored = colorizer->ColorizePointCloud();
  int non_red = 0;
  for (int i = 0; i < cloud_colored->points.size(); i++) {
    int r = cloud_colored->points[i].r;
    int g = cloud_colored->points[i].g;
    int b = cloud_colored->points[i].b;
    if (r != 0 && g != 0 && b != 0) {
      if (r != 254) { non_red++; }
    }
  }
  REQUIRE(non_red < 80);
  INFO(non_red);
}

TEST_CASE("Test factory method") {
  beam_colorize::ColorizerType PROJ = beam_colorize::ColorizerType::PROJECTION;
  beam_colorize::ColorizerType RAY = beam_colorize::ColorizerType::RAY_TRACE;

  auto projection = beam_colorize::Colorizer::Create(PROJ);
  auto raytrace = beam_colorize::Colorizer::Create(RAY);

  std::string proj_type(typeid(*projection).name());
  std::string ray_type(typeid(*raytrace).name());

  std::string proj_test = "Projection", ray_test = "RayTrace";

  REQUIRE(proj_type.find(proj_test) != -1);
  REQUIRE(ray_type.find(ray_test) != -1);
}

TEST_CASE("Test setter functions") {
  std::string test1_pcd = "test1/101_map.pcd";
  std::string intrinsics_name = "camera0.json";
  std::string test1_img = "test1/101_mask.jpg";
  // load intrinsics
  std::shared_ptr<beam_calibration::CameraModel> F1 =
      GetIntrinsics(intrinsics_name);
  // load Image
  cv::Mat image = GetImage(test1_img);
  // load pcd
  pcl::PointCloud<pcl::PointXYZ>::Ptr XYZ_cloud = GetPCD(test1_pcd);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr XYZRGB_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*XYZ_cloud, *XYZRGB_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  beam_colorize::Projection projection;
  REQUIRE_NOTHROW(projection.SetPointCloud(XYZRGB_cloud));
  REQUIRE_THROWS(projection.ColorizePointCloud());
  REQUIRE_NOTHROW(projection.SetPointCloud(XYZ_cloud));
  REQUIRE_NOTHROW(projection.SetImage(image));
  REQUIRE_NOTHROW(projection.SetIntrinsics(F1));

  beam_colorize::RayTrace raytrace;
  REQUIRE_NOTHROW(raytrace.SetPointCloud(XYZRGB_cloud));
  REQUIRE_THROWS(raytrace.ColorizePointCloud());
  REQUIRE_NOTHROW(raytrace.SetPointCloud(XYZ_cloud));
  REQUIRE_NOTHROW(raytrace.SetImage(image));
  REQUIRE_NOTHROW(raytrace.SetIntrinsics(F1));
}

TEST_CASE("Test correct raytrace colorization") {
  auto colorizer =
      beam_colorize::Colorizer::Create(beam_colorize::ColorizerType::RAY_TRACE);

  // load intrinsics
  std::string intrinsics_name = "camera0.json";
  std::string test1_pcd = "test1/101_map.pcd";
  std::string test2_pcd = "test2/259_map.pcd";
  std::string test1_img = "test1/101_mask.jpg";
  std::string test2_img = "test2/259_mask.jpg";
  std::shared_ptr<beam_calibration::CameraModel> model =
      GetIntrinsics(intrinsics_name);
  bool image_distorted = false;
  // Test case 1
  cv::Mat image = GetImage(test1_img);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = GetPCD(test1_pcd);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  colorizer->SetImage(image);
  colorizer->SetIntrinsics(model);
  colorizer->SetDistortion(image_distorted);
  colorizer->SetPointCloud(cloud);
  cloud_colored = colorizer->ColorizePointCloud();
  int non_red = 0;
  for (int i = 0; i < cloud_colored->points.size(); i++) {
    int r = cloud_colored->points[i].r;
    int g = cloud_colored->points[i].g;
    int b = cloud_colored->points[i].b;
    if (r != 0 && g != 0 && b != 0) {
      if (r != 254) { non_red++; }
    }
  }
  REQUIRE(non_red < 30);
  INFO(non_red);
}
