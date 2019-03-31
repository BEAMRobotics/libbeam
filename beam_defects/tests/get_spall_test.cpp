#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <pcl/io/pcd_io.h>

#include "beam_defects/Spall.h"

// SPALL TESTS

TEST_CASE("Spall defect type is returned", "[GetType]") {
  beam_defects::Spall spall{};

  REQUIRE(spall.GetType() == beam_defects::DefectType::SPALL);
  REQUIRE(spall.GetOSIMSeverity() == beam_defects::DefectOSIMSeverity::NONE);
}

TEST_CASE("Spall size calculation and VERY_SEVERE OSIM check", "[GetSize]") {
  auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  // Read in the pointcloud data
  pcl::PCDReader reader;
  reader.read("test_data/cloud_cluster_0.pcd", *cloud);

  // Instantiate the defect object with point cloud
  beam_defects::Spall spall{cloud};

  REQUIRE(spall.GetSize() == Approx(0.493669));
  REQUIRE(spall.GetOSIMSeverity() == beam_defects::DefectOSIMSeverity::VERY_SEVERE);
}

TEST_CASE("No spall returns size of 0") {
  beam_defects::Spall spall;

  REQUIRE(spall.GetSize() == Approx(0));
}

TEST_CASE("Spall (xy-plane) size calculation and LIGHT OSIM check") {
  auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->width = 6;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  // Generate some data
  cloud->points.push_back(pcl::PointXYZ{0,0,0});
  cloud->points.push_back(pcl::PointXYZ{0.1,0,0});
  cloud->points.push_back(pcl::PointXYZ{0.2,0,0});
  cloud->points.push_back(pcl::PointXYZ{0.25,0,0});
  cloud->points.push_back(pcl::PointXYZ{0.2,0,0.1});
  cloud->points.push_back(pcl::PointXYZ{0.1,0,0.1});
  
  beam_defects::Spall spall{cloud};

  REQUIRE(spall.GetSize() == Approx(0.0175));
  REQUIRE(spall.GetOSIMSeverity() == beam_defects::DefectOSIMSeverity::LIGHT);
}

TEST_CASE("Spall (xz-plane) size calculation and MEDIUM OSIM check") {
  auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->width = 11;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  // Generate some data
  cloud->points.push_back(pcl::PointXYZ{0,0,0});
  cloud->points.push_back(pcl::PointXYZ{0.1,0,0});
  cloud->points.push_back(pcl::PointXYZ{0.1,0,0.1});
  cloud->points.push_back(pcl::PointXYZ{0.1,0,0.2});
  cloud->points.push_back(pcl::PointXYZ{0.1,0,0.3});
  cloud->points.push_back(pcl::PointXYZ{0.1,0,0.4});
  cloud->points.push_back(pcl::PointXYZ{0.05,0,0.45});
  cloud->points.push_back(pcl::PointXYZ{0,0,0.4});
  cloud->points.push_back(pcl::PointXYZ{0,0,0.3});
  cloud->points.push_back(pcl::PointXYZ{0,0,0.2});
  cloud->points.push_back(pcl::PointXYZ{0,0,0.1});
  beam_defects::Spall spall{cloud};

  REQUIRE(spall.GetSize() == Approx(0.0425));
  REQUIRE(spall.GetOSIMSeverity() == beam_defects::DefectOSIMSeverity::MEDIUM);
}

TEST_CASE("Spall (yz-plane) size calculation and SEVERE OSIM check") {
  auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->width = 22;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  // Generate some data
  cloud->points.push_back(pcl::PointXYZ{0,0,0});
  cloud->points.push_back(pcl::PointXYZ{0,0.1,0});
  cloud->points.push_back(pcl::PointXYZ{0,0.2,0});
  cloud->points.push_back(pcl::PointXYZ{0,0.3,0});
  cloud->points.push_back(pcl::PointXYZ{0,0.4,0});
  cloud->points.push_back(pcl::PointXYZ{0,0.5,0});
  cloud->points.push_back(pcl::PointXYZ{0,0.5,-0.1});
  cloud->points.push_back(pcl::PointXYZ{0,0.4,-0.1});
  cloud->points.push_back(pcl::PointXYZ{0,0.3,-0.1});
  cloud->points.push_back(pcl::PointXYZ{0,0.2,-0.1});
  cloud->points.push_back(pcl::PointXYZ{0,0.1,-0.1});
  cloud->points.push_back(pcl::PointXYZ{0,0,-0.1});
  cloud->points.push_back(pcl::PointXYZ{0,-0.1,-0.1});
  cloud->points.push_back(pcl::PointXYZ{0,-0.2,-0.1});
  cloud->points.push_back(pcl::PointXYZ{0,-0.3,-0.1});
  cloud->points.push_back(pcl::PointXYZ{0,-0.4,-0.1});
  cloud->points.push_back(pcl::PointXYZ{0,-0.5,-0.1});
  cloud->points.push_back(pcl::PointXYZ{0,-0.5,0});
  cloud->points.push_back(pcl::PointXYZ{0,-0.4,0});
  cloud->points.push_back(pcl::PointXYZ{0,-0.3,0});
  cloud->points.push_back(pcl::PointXYZ{0,-0.2,0});
  cloud->points.push_back(pcl::PointXYZ{0,-0.1,0});

  beam_defects::Spall spall{cloud};

  REQUIRE(spall.GetSize() == Approx(0.1));
  REQUIRE(spall.GetOSIMSeverity() == beam_defects::DefectOSIMSeverity::SEVERE);
}
