#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <pcl/io/pcd_io.h>

#include "beam_defects/Crack.h"

// DELAMINATION TESTS

TEST_CASE("Crack defect type is returned", "[GetType]") {
  beam_defects::Crack crack{};

  REQUIRE(crack.GetType() == beam_defects::DefectType::CRACK);
}

TEST_CASE("Delam size calculation and VERY_SEVERE OSIM check", "[GetSize]") {
  auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  // Read in the pointcloud data
  pcl::PCDReader reader;
  reader.read("test_data/cloud_cluster_0.pcd", *cloud);

  // Instantiate the defect object with point cloud
  beam_defects::Crack crack{cloud};

  REQUIRE(crack.GetSize() == Approx(1.478998));
}

TEST_CASE("No crack returns size of 0") {
  beam_defects::Crack crack;

  REQUIRE_THROWS(crack.GetSize());
}

TEST_CASE("Crack (xy-plane) size calculation and LIGHT OSIM check") {
  auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->width = 6;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  // Generate some data
  cloud->points.push_back(pcl::PointXYZ{0, 0, 0});
  cloud->points.push_back(pcl::PointXYZ{0.1, 0, 0});
  cloud->points.push_back(pcl::PointXYZ{0.2, 0, 0});
  cloud->points.push_back(pcl::PointXYZ{0.25, 0, 0});
  cloud->points.push_back(pcl::PointXYZ{0.2, 0.1, 0});
  cloud->points.push_back(pcl::PointXYZ{0.1, 0.1, 0});

  beam_defects::Crack crack{cloud};

  REQUIRE(crack.GetSize() == Approx(0.269258));
}

TEST_CASE("Crack (xz-plane) size calculation and MEDIUM OSIM check") {
  auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->width = 11;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  // Generate some data
  cloud->points.push_back(pcl::PointXYZ{0, 0, 0});
  cloud->points.push_back(pcl::PointXYZ{0.1, 0, 0});
  cloud->points.push_back(pcl::PointXYZ{0.1, 0, 0.1});
  cloud->points.push_back(pcl::PointXYZ{0.1, 0, 0.2});
  cloud->points.push_back(pcl::PointXYZ{0.1, 0, 0.3});
  cloud->points.push_back(pcl::PointXYZ{0.1, 0, 0.4});
  cloud->points.push_back(pcl::PointXYZ{0.05, 0, 0.45});
  cloud->points.push_back(pcl::PointXYZ{0, 0, 0.4});
  cloud->points.push_back(pcl::PointXYZ{0, 0, 0.3});
  cloud->points.push_back(pcl::PointXYZ{0, 0, 0.2});
  cloud->points.push_back(pcl::PointXYZ{0, 0, 0.1});

  beam_defects::Crack crack{cloud};

  REQUIRE(crack.GetSize() == Approx(0.460977));
}

TEST_CASE("Crack (yz-plane) size calculation and SEVERE OSIM check") {
  auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->width = 22;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  // Generate some data
  cloud->points.push_back(pcl::PointXYZ{0, 0, 0});
  cloud->points.push_back(pcl::PointXYZ{0, 0.1, 0});
  cloud->points.push_back(pcl::PointXYZ{0, 0.2, 0});
  cloud->points.push_back(pcl::PointXYZ{0, 0.3, 0});
  cloud->points.push_back(pcl::PointXYZ{0, 0.4, 0});
  cloud->points.push_back(pcl::PointXYZ{0, 0.5, 0});
  cloud->points.push_back(pcl::PointXYZ{0, 0.5, -0.1});
  cloud->points.push_back(pcl::PointXYZ{0, 0.4, -0.1});
  cloud->points.push_back(pcl::PointXYZ{0, 0.3, -0.1});
  cloud->points.push_back(pcl::PointXYZ{0, 0.2, -0.1});
  cloud->points.push_back(pcl::PointXYZ{0, 0.1, -0.1});
  cloud->points.push_back(pcl::PointXYZ{0, 0, -0.1});
  cloud->points.push_back(pcl::PointXYZ{0, -0.1, -0.1});
  cloud->points.push_back(pcl::PointXYZ{0, -0.2, -0.1});
  cloud->points.push_back(pcl::PointXYZ{0, -0.3, -0.1});
  cloud->points.push_back(pcl::PointXYZ{0, -0.4, -0.1});
  cloud->points.push_back(pcl::PointXYZ{0, -0.5, -0.1});
  cloud->points.push_back(pcl::PointXYZ{0, -0.5, 0});
  cloud->points.push_back(pcl::PointXYZ{0, -0.4, 0});
  cloud->points.push_back(pcl::PointXYZ{0, -0.3, 0});
  cloud->points.push_back(pcl::PointXYZ{0, -0.2, 0});
  cloud->points.push_back(pcl::PointXYZ{0, -0.1, 0});

  beam_defects::Crack crack{cloud};

  REQUIRE(crack.GetSize() == Approx(1.00499));
}
