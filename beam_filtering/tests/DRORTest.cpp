#define CATCH_CONFIG_MAIN
#include "beam_filtering/DROR.h"
#include "beam_utils/math.hpp"
#include <catch2/catch.hpp>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr GetPCD() {
  std::string pcd_name = "snowy_scan.pcd";
  std::string pcd_location = __FILE__;
  pcd_location.erase(pcd_location.end() - 12, pcd_location.end());
  pcd_location += "test_data/" + pcd_name;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_location, *cloud) == -1) {
    LOG_INFO("Couldn't read pcd file:  %s\n", pcd_location.c_str());
  } else {
    LOG_INFO("Opened file: %s", pcd_location.c_str());
  }
  return cloud;
}

TEST_CASE("Test params i/o") {
  beam_filtering::DROR outlier_removal;
  double min_neighbors = 3, radius_multiplier = 3, azimuth_angle = 0.16,
         min_search_radius = 0.04;
  outlier_removal.SetRadiusMultiplier(radius_multiplier);
  outlier_removal.SetAzimuthAngle(azimuth_angle);
  outlier_removal.SetMinNeighbors(min_neighbors);
  outlier_removal.SetMinSearchRadius(min_search_radius);
  REQUIRE(outlier_removal.GetRadiusMultiplier() == radius_multiplier);
  REQUIRE(outlier_removal.GetAzimuthAngle() == azimuth_angle);
  REQUIRE(outlier_removal.GetMinNeighbors() == min_neighbors);
  REQUIRE(outlier_removal.GetMinSearchRadius() == min_search_radius);
}

TEST_CASE("Test with example scan") {
  PointCloud::Ptr input_cloud = GetPCD();
  PointCloud::Ptr output_cloud(new PointCloud);
  beam_filtering::DROR outlier_removal;
  double min_neighbors = 3, radius_multiplier = 3, azimuth_angle = 0.16,
         min_search_radius = 0.04;
  outlier_removal.SetRadiusMultiplier(radius_multiplier);
  outlier_removal.SetAzimuthAngle(azimuth_angle);
  outlier_removal.SetMinNeighbors(min_neighbors);
  outlier_removal.SetMinSearchRadius(min_search_radius);
  REQUIRE_NOTHROW(outlier_removal.Filter(input_cloud, *output_cloud));
  REQUIRE(output_cloud->points.size() == 33821);
  // std::cout << "input_cloud.points.size(): " << input_cloud->points.size() << "\n";
  // std::cout << "output_cloud.points.size(): " << output_cloud->points.size() << "\n";
}
