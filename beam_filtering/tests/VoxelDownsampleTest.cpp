#define CATCH_CONFIG_MAIN
#include "beam_filtering/VoxelDownsample.h"
#include "beam_utils/math.h"
#include <boost/filesystem.hpp>
#include <catch2/catch.hpp>
#include <pcl/io/pcd_io.h>

TEST_CASE("Testing voxeldownsample constructor and interface") {
  PointCloudXYZPtr input_cloud_ptr = std::make_shared<PointCloudXYZ>();
  PointCloudXYZPtr output_cloud_ptr = std::make_shared<PointCloudXYZ>();
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

PointCloudXYZPtr GetPCD() {
  std::string pcd_name = "snowy_scan.pcd";
  std::string pcd_location = __FILE__;
  pcd_location.erase(pcd_location.end() - 23, pcd_location.end());
  pcd_location += "test_data/" + pcd_name;
  PointCloudXYZPtr cloud = std::make_shared<PointCloudXYZ>();
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_location, *cloud) == -1) {
    LOG_INFO("Couldn't read pcd file:  %s\n", pcd_location.c_str());
  } else {
    LOG_INFO("Opened file: %s", pcd_location.c_str());
  }
  return cloud;
}

PointCloudXYZPtr CreateUniformDensityPointCloud(const Eigen::Vector3f& min_point, const Eigen::Vector3f& max_point, const float density){
  PointCloudXYZPtr output_cloud_ptr = std::make_shared<PointCloudXYZ>();
  for (float x = min_point[0]; x < max_point[0]; x+=density) {
    for (float y = min_point[1]; y < max_point[1]; y+=density) {
      for (float z = min_point[2]; z < max_point[2]; z+=density) {
        pcl::PointXYZ p;
        p.x = x;
        p.y = y;
        p.z = z;
        output_cloud_ptr->push_back(p);
      }
    }
  }
  return output_cloud_ptr;
}

TEST_CASE("Testing voxeldownsample pointcloud I/O w/ real scan and generated uniform density clouds.") {
  PointCloudXYZPtr scan_input_cloud_ptr = GetPCD();
  PointCloudXYZPtr scan_output_cloud_ptr = std::make_shared<PointCloudXYZ>();
  Eigen::Vector3f scan_voxel_size(.5, .5, .5);
  beam_filtering::VoxelDownsample downsampler(scan_voxel_size);
  REQUIRE_NOTHROW(downsampler.Filter(*scan_input_cloud_ptr, *scan_output_cloud_ptr));
  REQUIRE(scan_output_cloud_ptr->points.size() < scan_input_cloud_ptr->points.size());

  Eigen::Vector3f min_point(0,0,0);
  Eigen::Vector3f max_point(10,10,10);
  PointCloudXYZPtr uniform_input_cloud_ptr = CreateUniformDensityPointCloud(min_point, max_point, .2);
  PointCloudXYZPtr uniform_output_cloud_ptr = std::make_shared<PointCloudXYZ>();
  REQUIRE_NOTHROW(downsampler.Filter(*uniform_input_cloud_ptr, *uniform_output_cloud_ptr));
  REQUIRE(uniform_output_cloud_ptr->points.size()/uniform_input_cloud_ptr->points.size() < .1);
}