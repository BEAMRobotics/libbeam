#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <pcl/io/pcd_io.h>

#include "beam_defects/Delam.h"

// DELAMINATION TESTS

TEST_CASE("Delam defect type is returned", "[GetType]") {
  beam_defects::Delam delam{};

  REQUIRE(delam.GetType() == beam_defects::DefectType::DELAM);
  REQUIRE(delam.GetOSIMSeverity() == beam_defects::DefectOSIMSeverity::NONE);
}

TEST_CASE("Delam size calculation and VERY_SEVERE OSIM check", "[GetSize]") {
  auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto cloud2 = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  // Read in the pointcloud data
  pcl::PCDReader reader;
  reader.read("test_data/cloud_cluster_0.pcd", *cloud);
  reader.read("test_data/cloud_cluster_1.pcd", *cloud2);

  // Instantiate the defect object with point cloud
  beam_defects::Delam delam{cloud}, delam2{cloud2};

  REQUIRE(delam.GetSize() == Approx(0.4803934));
  REQUIRE(delam.GetOSIMSeverity() ==
          beam_defects::DefectOSIMSeverity::VERY_SEVERE);
  REQUIRE(delam2.GetSize() == Approx(0.23996));
  REQUIRE(delam2.GetOSIMSeverity() == beam_defects::DefectOSIMSeverity::SEVERE);
}

TEST_CASE("No Delam returns size of 0") {
  beam_defects::Delam delam;

  REQUIRE(delam.GetSize() == Approx(0));
}

TEST_CASE("Delam (xy-plane) size calculation and LIGHT OSIM check") {
  auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->width = 6;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  // Generate a polygonal shape
  cloud->points.push_back(pcl::PointXYZ{0, 0, 0});
  cloud->points.push_back(pcl::PointXYZ{0.1, 0, 0});
  cloud->points.push_back(pcl::PointXYZ{0.2, 0, 0});
  cloud->points.push_back(pcl::PointXYZ{0.25, 0, 0});
  cloud->points.push_back(pcl::PointXYZ{0.2, 0.1, 0});
  cloud->points.push_back(pcl::PointXYZ{0.1, 0.1, 0});

  beam_defects::Delam delam{cloud};

  REQUIRE(delam.GetSize() == Approx(0.0175));
  REQUIRE(delam.GetOSIMSeverity() == beam_defects::DefectOSIMSeverity::LIGHT);
}

TEST_CASE("Delam (xz-plane) size calculation and MEDIUM OSIM check") {
  auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->width = 11;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  // Generate a polygonal shape
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

  beam_defects::Delam delam{cloud};

  REQUIRE(delam.GetSize() == Approx(0.0425));
  REQUIRE(delam.GetOSIMSeverity() == beam_defects::DefectOSIMSeverity::MEDIUM);
}

TEST_CASE("Delam (yz-plane) size calculation and SEVERE OSIM check") {
  auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->width = 22;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  // Generate a polygonal shape
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

  beam_defects::Delam delam{cloud};

  REQUIRE(delam.GetSize() == Approx(0.1));
  REQUIRE(delam.GetOSIMSeverity() == beam_defects::DefectOSIMSeverity::SEVERE);
}
