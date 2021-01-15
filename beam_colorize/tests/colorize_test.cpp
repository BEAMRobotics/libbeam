#define CATCH_CONFIG_MAIN

#include <iostream>
#include <typeinfo>

#include <boost/filesystem.hpp>
#include <catch2/catch.hpp>
#include <nlohmann/json.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <beam_calibration/Radtan.h>
#include <beam_colorize/Projection.h>
#include <beam_colorize/RayTrace.h>

std::unique_ptr<beam_colorize::Colorizer> colorizer_;
std::shared_ptr<beam_calibration::CameraModel> camera_model_;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>);
cv::Mat image_;

void LoadColorizer(beam_colorize::ColorizerType type) {
  std::string current_file_path = "colorize_test.cpp";
  std::string cur_dir = __FILE__;
  cur_dir.erase(cur_dir.end() - current_file_path.length(), cur_dir.end());
  colorizer_ = beam_colorize::Colorizer::Create(type);
  // load camera model
  std::string intrinsics_location = cur_dir + "test_data/camera0.json";
  camera_model_ =
      std::make_shared<beam_calibration::Radtan>(intrinsics_location);
  // load point cloud
  std::string cloud_location = cur_dir + "test_data/259_map.pcd";
  pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_location, *cloud_);
  // load image
  std::string image_location = cur_dir + "test_data/259_mask.jpg";
  image_ = cv::imread(image_location, CV_LOAD_IMAGE_COLOR);
  // init variables
  bool image_distorted = true;
  colorizer_->SetPointCloud(cloud_);
  colorizer_->SetImage(image_);
  colorizer_->SetIntrinsics(camera_model_);
  colorizer_->SetDistortion(image_distorted);
}

TEST_CASE("Test correct projection colorization") {
  LoadColorizer(beam_colorize::ColorizerType::PROJECTION);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  cloud_colored = colorizer_->ColorizePointCloud();

  int non_red = 0;
  for (int i = 0; i < cloud_colored->points.size(); i++) {
    int r = cloud_colored->points[i].r;
    int g = cloud_colored->points[i].g;
    int b = cloud_colored->points[i].b;
    if (r != 0 && g != 0 && b != 0) {
      if (r != 254) { non_red++; }
    }
  }
  REQUIRE(non_red < 300);
  INFO(non_red);
}

TEST_CASE("Test correct raytrace colorization") {
  LoadColorizer(beam_colorize::ColorizerType::RAY_TRACE);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  cloud_colored = colorizer_->ColorizePointCloud();

  int non_red = 0;
  for (int i = 0; i < cloud_colored->points.size(); i++) {
    int r = cloud_colored->points[i].r;
    int g = cloud_colored->points[i].g;
    int b = cloud_colored->points[i].b;
    if (r != 0 && g != 0 && b != 0) {
      if (r != 254) { non_red++; }
    }
  }
  REQUIRE(non_red < 300);
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
  std::string cur_dir = __FILE__;
  cur_dir.erase(cur_dir.end() - 23, cur_dir.end());
  cur_dir += "tests/test_data/";
  // load camera intrinsics
  std::string intrinsics_loc = cur_dir + "camera0.json";
  std::shared_ptr<beam_calibration::CameraModel> model =
      std::make_shared<beam_calibration::Radtan>(intrinsics_loc);
  // load pcd file
  pcl::PointCloud<pcl::PointXYZ>::Ptr XYZ_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  std::string cloud_loc = cur_dir + "259_map.pcd";
  pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_loc, *XYZ_cloud);
  // load Image
  std::string image_location = cur_dir + "259_mask.jpg";
  cv::Mat image;
  image = cv::imread(image_location, CV_LOAD_IMAGE_COLOR);

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
  REQUIRE_NOTHROW(projection.SetIntrinsics(model));

  beam_colorize::RayTrace raytrace;
  REQUIRE_NOTHROW(raytrace.SetPointCloud(XYZRGB_cloud));
  REQUIRE_THROWS(raytrace.ColorizePointCloud());
  REQUIRE_NOTHROW(raytrace.SetPointCloud(XYZ_cloud));
  REQUIRE_NOTHROW(raytrace.SetImage(image));
  REQUIRE_NOTHROW(raytrace.SetIntrinsics(model));
}
