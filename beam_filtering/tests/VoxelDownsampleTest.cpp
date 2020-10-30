#define CATCH_CONFIG_MAIN
#include "beam_filtering/VoxelDownsample.h"
#include "beam_utils/math.hpp"
#include <boost/filesystem.hpp>
#include <catch2/catch.hpp>
#include <pcl/io/pcd_io.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

TEST_CASE("Test i/o") {
  PointCloud::Ptr input_cloud_ptr = boost::make_shared<PointCloud>();
  PointCloud::Ptr output_cloud_ptr = boost::make_shared<PointCloud>();
  // Test constructor requires voxel size arg.
  Eigen::Vector3f voxel_size(.5, .5, .5);
  beam_filtering::VoxelDownsample downsampler(voxel_size);
  // Test constructor works.
  REQUIRE(downsampler.GetVoxelSize() == voxel_size);
  Eigen::Vector3f new_voxel_size(.3, .3, .3);
  downsampler.SetVoxelSize(new_voxel_size);
  // Test SetVoxelSize works.
  REQUIRE(downsampler.GetVoxelSize() == new_voxel_size);
  REQUIRE_NOTHROW(downsampler.Filter(*input_cloud_ptr, *output_cloud_ptr));
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GetPCD() {
  std::string pcd_name = "snowy_scan.pcd";
  std::string pcd_location = __FILE__;
  pcd_location.erase(pcd_location.end() - 23, pcd_location.end());
  pcd_location += "test_data/" + pcd_name;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<PointCloud>();
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_location, *cloud) == -1) {
    LOG_INFO("Couldn't read pcd file:  %s\n", pcd_location.c_str());
  } else {
    LOG_INFO("Opened file: %s", pcd_location.c_str());
  }
  return cloud;
}

TEST_CASE("Test filtering with example scan.") {
  PointCloud::Ptr input_cloud_ptr = GetPCD();
  PointCloud::Ptr output_cloud_ptr = boost::make_shared<PointCloud>();
  Eigen::Vector3f voxel_size(.5, .5, .5);
  beam_filtering::VoxelDownsample downsampler(voxel_size);
  REQUIRE_NOTHROW(downsampler.Filter(*input_cloud_ptr, *output_cloud_ptr));
  REQUIRE(output_cloud_ptr->points.size() < input_cloud_ptr->points.size());
}