#define CATCH_CONFIG_MAIN

#include <boost/filesystem.hpp>
#include <catch2/catch.hpp>
#include <pcl/io/pcd_io.h>

#include <beam_filtering/VoxelDownsample.h>

PointCloudPtr GetPCD() {
  std::string pcd_name = "snowy_scan.pcd";
  std::string pcd_location = __FILE__;
  pcd_location.erase(pcd_location.end() - 23, pcd_location.end());
  pcd_location += "test_data/" + pcd_name;
  PointCloudPtr cloud = std::make_shared<PointCloud>();
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_location, *cloud) == -1) {
    LOG_INFO("Couldn't read pcd file:  %s\n", pcd_location.c_str());
  } else {
    LOG_INFO("Opened file: %s", pcd_location.c_str());
  }
  return cloud;
}

PointCloudPtr CreateUniformDensityPointCloud(const Eigen::Vector3f& min_point,
                                             const Eigen::Vector3f& max_point,
                                             const float density) {
  PointCloudPtr output_cloud_ptr = std::make_shared<PointCloud>();
  for (float x = min_point[0]; x < max_point[0]; x += density) {
    for (float y = min_point[1]; y < max_point[1]; y += density) {
      for (float z = min_point[2]; z < max_point[2]; z += density) {
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

TEST_CASE("Testing voxeldownsample constructor and interface") {
  PointCloudPtr input_cloud_ptr = std::make_shared<PointCloud>();
  PointCloudPtr output_cloud_ptr = std::make_shared<PointCloud>();

  // Test constructor requires voxel size arg.
  Eigen::Vector3f voxel_size(.5, .5, .5);
  beam_filtering::VoxelDownsample<> downsampler(voxel_size);

  // Test constructor works.
  REQUIRE(downsampler.GetVoxelSize() == voxel_size);
  Eigen::Vector3f new_voxel_size(.3, .3, .3);
  downsampler.SetVoxelSize(new_voxel_size);

  // Test SetVoxelSize works.
  REQUIRE(downsampler.GetVoxelSize() == new_voxel_size);
  downsampler.SetInputCloud(input_cloud_ptr);
  REQUIRE_NOTHROW(downsampler.Filter());
}

TEST_CASE("Testing voxeldownsample pointcloud I/O w/ real scan and generated "
          "uniform density clouds.") {
  PointCloudPtr scan_input_cloud_ptr = GetPCD();
  Eigen::Vector3f scan_voxel_size(.5, .5, .5);
  beam_filtering::VoxelDownsample<> downsampler(scan_voxel_size);
  downsampler.SetInputCloud(scan_input_cloud_ptr);

  REQUIRE_NOTHROW(downsampler.Filter());
  PointCloud scan_output_cloud = downsampler.GetFilteredCloud();
  REQUIRE(scan_output_cloud.points.size() <
          scan_input_cloud_ptr->points.size());

  Eigen::Vector3f min_point(0, 0, 0);
  Eigen::Vector3f max_point(10, 10, 10);
  PointCloudPtr uniform_input_cloud_ptr =
      CreateUniformDensityPointCloud(min_point, max_point, .2);

  downsampler.SetInputCloud(uniform_input_cloud_ptr);
  REQUIRE_NOTHROW(downsampler.Filter());
  PointCloud uniform_output_cloud = downsampler.GetFilteredCloud();
  REQUIRE(uniform_output_cloud.points.size() /
              uniform_input_cloud_ptr->points.size() <
          .1);
}