#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>

#include <beam_filtering/DROR.h>

PointCloudPtr GetPCD() {
  std::string pcd_name = "snowy_scan.pcd";
  std::string pcd_location = __FILE__;
  pcd_location.erase(pcd_location.end() - 12, pcd_location.end());
  pcd_location += "test_data/" + pcd_name;
  PointCloudPtr cloud = std::make_shared<PointCloud>();
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_location, *cloud) == -1) {
    LOG_INFO("Couldn't read pcd file:  %s\n", pcd_location.c_str());
  } else {
    LOG_INFO("Opened file: %s", pcd_location.c_str());
  }
  return cloud;
}

TEST_CASE("Test params i/o") {
  beam_filtering::DROR<> outlier_removal;
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

  outlier_removal = beam_filtering::DROR<>(radius_multiplier, azimuth_angle,
                                           min_neighbors, min_search_radius);
  REQUIRE(outlier_removal.GetRadiusMultiplier() == radius_multiplier);
  REQUIRE(outlier_removal.GetAzimuthAngle() == azimuth_angle);
  REQUIRE(outlier_removal.GetMinNeighbors() == min_neighbors);
  REQUIRE(outlier_removal.GetMinSearchRadius() == min_search_radius);
}

TEST_CASE("Test with example scan") {
  PointCloudPtr input_cloud = GetPCD();
  beam_filtering::DROR<> outlier_removal;
  double min_neighbors = 3, radius_multiplier = 3, azimuth_angle = 0.16,
         min_search_radius = 0.04;
  outlier_removal.SetRadiusMultiplier(radius_multiplier);
  outlier_removal.SetAzimuthAngle(azimuth_angle);
  outlier_removal.SetMinNeighbors(min_neighbors);
  outlier_removal.SetMinSearchRadius(min_search_radius);
  outlier_removal.SetInputCloud(input_cloud);

  REQUIRE_NOTHROW(outlier_removal.Filter());
  PointCloud output_cloud = outlier_removal.GetFilteredCloud();

  REQUIRE(output_cloud.points.size() == 33821);
}
