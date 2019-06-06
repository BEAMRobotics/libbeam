#include "beam_calibration/CameraModel.h"
#include "beam_calibration/TfTree.h"
#include "beam_colorize/Projection.h"
#include "beam_colorize/RayTrace.h"
#include "beam_utils/math.hpp"

#include <boost/filesystem.hpp>

#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

int main() {
  // load intrinsics
  std::string intrinsics_name = "F1.json";
  std::string intrinsics_location = __FILE__;
  intrinsics_location.erase(intrinsics_location.end() - 15,
                            intrinsics_location.end());
  intrinsics_location += "tests/test_data/";
  intrinsics_location += intrinsics_name;

  std::shared_ptr<beam_calibration::CameraModel> F1 =
      beam_calibration::CameraModel::LoadJSON(intrinsics_location);

  // load Image
  std::string image_name = "image18reduced.jpg";
  std::string image_location = __FILE__;
  image_location.erase(image_location.end() - 15, image_location.end());
  image_location += "tests/test_data/";
  image_location += image_name;

  cv::Mat image;
  image = cv::imread(image_location, CV_LOAD_IMAGE_COLOR);

  if (!image.data) {
    BEAM_INFO("Could not open or find the image: {}", image_location);
    return -1;
  } else {
    BEAM_INFO("Opened file: {}", image_location);
  }

  // load pcd
  std::string pcd_name = "map18crop.pcd";
  std::string pcd_location = __FILE__;
  pcd_location.erase(pcd_location.end() - 15, pcd_location.end());
  pcd_location += "tests/test_data/";
  pcd_location += pcd_name;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_location, *cloud) == -1) {
    BEAM_INFO("Couldn't read pcd file:  {}\n", pcd_location);
    return (-1);
  } else {
    BEAM_INFO("Opened file: {}", pcd_location);
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  beam_colorize::RayTrace colorizer;
  bool image_distorted = true;
  colorizer.SetPointCloud(cloud);
  colorizer.SetImage(image);
  colorizer.SetIntrinsics(F1);
  colorizer.SetDistortion(image_distorted);
  colorizer.CorrectImageGamma();
  cloud_colored = colorizer.ColorizePointCloud();

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(200, 200, 200);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
      cloud_colored);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud_colored, rgb, "test cloud");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "test cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  while (!viewer->wasStopped()) { viewer->spinOnce(10); }

  // pcl::io::savePCDFileASCII ("/home/nick/test_pcd.pcd", *cloud_colored);
}
