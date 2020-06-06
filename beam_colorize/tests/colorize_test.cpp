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

pcl::PointCloud<pcl::PointXYZ>::Ptr GetPCD() {
  std::string pcd_name = "map18crop.pcd";
  std::string pcd_location = __FILE__;
  pcd_location.erase(pcd_location.end() - 17, pcd_location.end());
  pcd_location += "test_data/" + pcd_name;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_location, *cloud) == -1) {
    BEAM_INFO("Couldn't read pcd file:  {}\n", pcd_location);
  } else {
    BEAM_INFO("Opened file: {}", pcd_location);
  }
  return cloud;
}

cv::Mat GetImage() {
  std::string image_name = "test_img.jpg";
  std::string image_location = __FILE__;
  image_location.erase(image_location.end() - 17, image_location.end());
  image_location += "test_data/" + image_name;
  cv::Mat image;
  image = cv::imread(image_location, CV_LOAD_IMAGE_COLOR);
  if (!image.data) {
    std::cout << "Could not open or find the image" << std::endl;
  } else {
    BEAM_INFO("Opened file: {}", image_location);
  }
  return image;
}

std::shared_ptr<beam_calibration::CameraModel> GetIntrinsics() {
  // load intrinsics
  std::string intrinsics_name = "F1.json";
  std::string intrinsics_location = __FILE__;
  intrinsics_location.erase(intrinsics_location.end() - 17,
                            intrinsics_location.end());
  intrinsics_location += "test_data/" + intrinsics_name;
  std::shared_ptr<beam_calibration::CameraModel> F1 =
      std::make_shared<beam_calibration::Radtan>(intrinsics_location);
  return F1;
}

TEST_CASE("Test correct projection colorization and exceptions") {
  // load intrinsics
  std::shared_ptr<beam_calibration::CameraModel> F1 = GetIntrinsics();
  // load Image
  cv::Mat image = GetImage();
  // load pcd
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = GetPCD();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  auto colorizer = beam_colorize::Colorizer::Create(
      beam_colorize::ColorizerType::PROJECTION);
  bool image_distorted = true;

  colorizer->SetPointCloud(empty_cloud);
  REQUIRE_THROWS(colorizer->ColorizePointCloud());
  colorizer->SetImage(image);
  REQUIRE_THROWS(colorizer->ColorizePointCloud());
  colorizer->SetIntrinsics(F1);
  REQUIRE_THROWS(colorizer->ColorizePointCloud());
  colorizer->SetDistortion(image_distorted);
  colorizer->SetPointCloud(cloud);
  REQUIRE_NOTHROW(colorizer->ColorizePointCloud());
  cloud_colored = colorizer->ColorizePointCloud();

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
  // load intrinsics
  std::shared_ptr<beam_calibration::CameraModel> F1 = GetIntrinsics();
  // load Image
  cv::Mat image = GetImage();
  // load pcd
  pcl::PointCloud<pcl::PointXYZ>::Ptr XYZ_cloud = GetPCD();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr XYZRGB_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*XYZ_cloud, *XYZRGB_cloud);

  beam_colorize::Projection projection;
  REQUIRE_NOTHROW(projection.SetPointCloud(XYZ_cloud));
  REQUIRE_NOTHROW(projection.SetPointCloud(XYZRGB_cloud));
  REQUIRE_NOTHROW(projection.SetImage(image));
  REQUIRE_NOTHROW(projection.SetIntrinsics(F1));

  beam_colorize::RayTrace raytrace;
  REQUIRE_NOTHROW(raytrace.SetPointCloud(XYZ_cloud));
  REQUIRE_NOTHROW(raytrace.SetPointCloud(XYZRGB_cloud));
  REQUIRE_NOTHROW(raytrace.SetImage(image));
  REQUIRE_NOTHROW(raytrace.SetIntrinsics(F1));
}

TEST_CASE("Test correct raytrace colorization and exceptions") {
  // load intrinsics
  std::shared_ptr<beam_calibration::CameraModel> F1 = GetIntrinsics();
  // load Image
  cv::Mat image = GetImage();
  // load pcd
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = GetPCD();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  auto colorizer =
      beam_colorize::Colorizer::Create(beam_colorize::ColorizerType::RAY_TRACE);
  bool image_distorted = true;

  colorizer->SetPointCloud(empty_cloud);
  REQUIRE_THROWS(colorizer->ColorizePointCloud());
  colorizer->SetImage(image);
  REQUIRE_THROWS(colorizer->ColorizePointCloud());
  colorizer->SetIntrinsics(F1);
  REQUIRE_THROWS(colorizer->ColorizePointCloud());
  colorizer->SetDistortion(image_distorted);
  colorizer->SetPointCloud(cloud);
  REQUIRE_NOTHROW(colorizer->ColorizePointCloud());
  cloud_colored = colorizer->ColorizePointCloud();

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

  // require that it correclty colors over 20% of the points projection colors
  // 20% occurs when dilation is 1, and increases as dilation does
  REQUIRE(percent_red > 0.20);
  REQUIRE(percent_blue > 0.20);
}
