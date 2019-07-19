#define CATCH_CONFIG_MAIN
#include <beam_containers/PointBridge.h>
#include <catch2/catch.hpp>
#include <pcl/io/pcd_io.h>

#include "beam_defects/Corrosion.h"
#include "beam_defects/extract_functions.h"

using PB = beam_containers::PointBridge;

// CORROSION TESTS

TEST_CASE("Corrosion defect type is returned", "[GetType]") {
  beam_defects::Corrosion corrosion{};

  REQUIRE(corrosion.GetType() == beam_defects::DefectType::CORROSION);
  REQUIRE_THROWS(corrosion.GetOSIMSeverity());
  REQUIRE_THROWS(corrosion.GetMaxDim2D());
}

TEST_CASE("Corrosion size calculation and VERY_SEVERE OSIM check",
          "[GetSize]") {
  auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto cloud2 = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  // Read in the pointcloud data
  pcl::PCDReader reader;
  reader.read("test_data/cloud_cluster_0.pcd", *cloud);
  reader.read("test_data/cloud_cluster_1.pcd", *cloud2);

  // Instantiate the defect object with point cloud
  beam_defects::Corrosion corrosion{cloud}, corrosion2;
  REQUIRE_THROWS(corrosion2.GetSize());
  corrosion2.SetPointCloud(cloud2);

  REQUIRE(corrosion.GetSize() == Approx(0.4803934));
  REQUIRE(corrosion.GetOSIMSeverity() ==
          beam_defects::DefectOSIMSeverity::VERY_SEVERE);
  REQUIRE(corrosion.GetMaxDim2D() == Approx(1.32481));
  REQUIRE(corrosion2.GetSize() == Approx(0.23996));
  REQUIRE(corrosion2.GetOSIMSeverity() ==
          beam_defects::DefectOSIMSeverity::SEVERE);
  REQUIRE(corrosion2.GetMaxDim2D() == Approx(0.733556));

  // Test SetHullAlpha()
  float alpha = 0.2;
  corrosion.SetHullAlpha(alpha);
  REQUIRE(corrosion.GetSize() == Approx(0.531694));
  REQUIRE(corrosion.GetSize() == Approx(0.531694));
  corrosion.SetHullAlpha(); // sets as default (0.1)
  REQUIRE(corrosion.GetSize() == Approx(0.4803934));
}

TEST_CASE("No Corrosion returns size of 0") {
  beam_defects::Corrosion corrosion;

  REQUIRE_THROWS(corrosion.GetSize());
  REQUIRE_THROWS(corrosion.GetOSIMSeverity());
  REQUIRE_THROWS(corrosion.GetMaxDim2D());
}

TEST_CASE("Corrosion (xy-plane) extraction, size calculation, and LIGHT OSIM "
          "severity check") {
  auto cloud = boost::make_shared<pcl::PointCloud<PB>>();
  cloud->width = 10;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);
  // Add default values for point bridge
  // x,y,z,intensity,rgb,r,g,b,a,thermal,crack,spall,corrosion,delam
  float RGB = 0, I = 0, C = 0, S = 0, CS1 = 0.9, CS2 = 0, D = 0;
  uint8_t T = 0, R = 0, G = 0, B = 0, A = 0;

  // Generate shape in xy with points to represent corrosion
  cloud->points[0] = PB{0, 0, 0, I, RGB, R, G, B, A, T, C, S, CS1, D};
  cloud->points[1] = PB{0.1, 0, 0, I, RGB, R, G, B, A, T, C, S, CS1, D};
  cloud->points[2] = PB{0.2, 0, 0, I, RGB, R, G, B, A, T, C, S, CS1, D};
  cloud->points[3] = PB{0.25, 0, 0, I, RGB, R, G, B, A, T, C, S, CS1, D};
  cloud->points[4] = PB{0.05, 0.05, 0, I, RGB, R, G, B, A, T, C, S, CS1, D};
  cloud->points[5] = PB{0.2, 0.1, 0, I, RGB, R, G, B, A, T, C, S, CS1, D};
  cloud->points[6] = PB{0.1, 0.1, 0, I, RGB, R, G, B, A, T, C, S, CS1, D};

  // Add extra points that aren't labelled as corrosion
  cloud->points[7] = PB{1.4, .3, 0, I, RGB, R, G, B, A, T, C, S, CS2, D};
  cloud->points[8] = PB{0.8, 0.2, 0, I, RGB, R, G, B, A, T, C, S, CS2, D};
  cloud->points[9] = PB{-.1, -.1, 0, I, RGB, R, G, B, A, T, C, S, CS2, D};

  // Extract XYZ point cloud
  auto cloud_xyz = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  float threshold = 0.8; // threshold for determining if point it corrosion
  *cloud_xyz = beam_defects::IsolateCorrosionPoints(cloud, threshold);
  REQUIRE(cloud_xyz->points.size() == Approx(7));

  // Create a corrosion object
  beam_defects::Corrosion corrosion{cloud_xyz};
  corrosion.SetHullAlpha(); // Test setting hull alpha to same value as initial
  REQUIRE(corrosion.GetSize() == Approx(0.0175));
  REQUIRE(corrosion.GetOSIMSeverity() ==
          beam_defects::DefectOSIMSeverity::LIGHT);
  REQUIRE(corrosion.GetMaxDim2D() == Approx(0.25));
}

TEST_CASE("Corrosion (xz-plane and yz-plane) extraction, size calculation, and "
          "MEDIUM OSIM check") {
  auto cloud = boost::make_shared<pcl::PointCloud<PB>>();
  cloud->width = 40;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);
  // Add default values for point bridge
  // x,y,z,intensity,rgb,r,g,b,a,thermal,crack,spall,corrosion,delam
  float RGB = 0, I = 0, C = 0, S = 0, CS1 = 0.95, CS2 = 0.75, CS3 = 0.25, D = 0;
  uint8_t T = 0, R = 0, G = 0, B = 0, A = 0;

  // Generate shape in xz plane with points to represent a corrosion
  cloud->points[0] = PB{0, 0, 0, I, RGB, R, G, B, A, T, C, S, CS1, D};
  cloud->points[1] = PB{0.1, 0, 0, I, RGB, R, G, B, A, T, C, S, CS2, D};
  cloud->points[2] = PB{0.1, 0, 0.1, I, RGB, R, G, B, A, T, C, S, CS1, D};
  cloud->points[3] = PB{0.1, 0, 0.2, I, RGB, R, G, B, A, T, C, S, CS1, D};
  cloud->points[4] = PB{0.05, 0, 0.05, I, RGB, R, G, B, A, T, C, S, CS2, D};
  cloud->points[5] = PB{0.1, 0, 0.3, I, RGB, R, G, B, A, T, C, S, CS2, D};
  cloud->points[6] = PB{0.1, 0, 0.4, I, RGB, R, G, B, A, T, C, S, CS2, D};
  cloud->points[7] = PB{0.025, 0, 0.05, I, RGB, R, G, B, A, T, C, S, CS1, D};
  cloud->points[8] = PB{0.05, 0, 0.45, I, RGB, R, G, B, A, T, C, S, CS2, D};
  cloud->points[9] = PB{0, 0, 0.4, I, RGB, R, G, B, A, T, C, S, CS1, D};
  cloud->points[10] = PB{0, 0, 0.3, I, RGB, R, G, B, A, T, C, S, CS1, D};
  cloud->points[11] = PB{0, 0, 0.2, I, RGB, R, G, B, A, T, C, S, CS1, D};
  cloud->points[12] = PB{0, 0, 0.1, I, RGB, R, G, B, A, T, C, S, CS2, D};

  // Generate Shape in yz plane with points to represent a corrosion
  cloud->points[13] = PB{5, 0, 0, I, RGB, R, G, B, A, T, C, S, CS1, D};
  cloud->points[14] = PB{5, 0.1, 0, I, RGB, R, G, B, A, T, C, S, CS2, D};
  cloud->points[15] = PB{5, 0.2, 0, I, RGB, R, G, B, A, T, C, S, CS1, D};
  cloud->points[16] = PB{5, 0.3, 0, I, RGB, R, G, B, A, T, C, S, CS2, D};
  cloud->points[17] = PB{5, 0.4, 0, I, RGB, R, G, B, A, T, C, S, CS2, D};
  cloud->points[18] = PB{5, 0.5, 0, I, RGB, R, G, B, A, T, C, S, CS1, D};
  cloud->points[19] = PB{5, 0.5, -0.1, I, RGB, R, G, B, A, T, C, S, CS2, D};
  cloud->points[20] = PB{5, 0.4, -0.1, I, RGB, R, G, B, A, T, C, S, CS1, D};
  cloud->points[21] = PB{5, 0.3, -0.1, I, RGB, R, G, B, A, T, C, S, CS2, D};
  cloud->points[22] = PB{5, 0.2, -0.1, I, RGB, R, G, B, A, T, C, S, CS1, D};
  cloud->points[23] = PB{5, 0.1, -0.1, I, RGB, R, G, B, A, T, C, S, CS1, D};
  cloud->points[24] = PB{5, 0, -0.1, I, RGB, R, G, B, A, T, C, S, CS1, D};
  cloud->points[25] = PB{5, -0.1, -0.1, I, RGB, R, G, B, A, T, C, S, CS2, D};
  cloud->points[26] = PB{5, -0.2, -0.1, I, RGB, R, G, B, A, T, C, S, CS2, D};
  cloud->points[27] = PB{5, -0.3, -0.1, I, RGB, R, G, B, A, T, C, S, CS1, D};
  cloud->points[28] = PB{5, -0.4, -0.1, I, RGB, R, G, B, A, T, C, S, CS2, D};
  cloud->points[29] = PB{5, -0.5, -0.1, I, RGB, R, G, B, A, T, C, S, CS1, D};
  cloud->points[30] = PB{5, -0.5, 0, I, RGB, R, G, B, A, T, C, S, CS2, D};
  cloud->points[31] = PB{5, -0.4, 0, I, RGB, R, G, B, A, T, C, S, CS2, D};
  cloud->points[32] = PB{5, -0.3, 0, I, RGB, R, G, B, A, T, C, S, CS2, D};
  cloud->points[33] = PB{5, -0.2, 0, I, RGB, R, G, B, A, T, C, S, CS1, D};
  cloud->points[34] = PB{5, -0.1, 0, I, RGB, R, G, B, A, T, C, S, CS1, D};

  // Add extra points that aren't labelled as corrosion
  cloud->points[35] = PB{0.4, 0.1, 0.1, I, RGB, R, G, B, A, T, C, S, CS3, D};
  cloud->points[36] = PB{0.8, 0.2, 0.7, I, RGB, R, G, B, A, T, C, S, CS3, D};
  cloud->points[37] = PB{-.1, -.1, -0.3, I, RGB, R, G, B, A, T, C, S, CS3, D};
  cloud->points[38] = PB{10, 10, 5, I, RGB, R, G, B, A, T, C, S, CS3, D};
  cloud->points[39] = PB{-10, -10, -5, I, RGB, R, G, B, A, T, C, S, CS3, D};

  // Generate vector of delam objects to test GetCorrosion
  auto cloud_xyz = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  float threshold = 0.5, tolerance = 0.2;
  int min_points = 5, max_points = 1000;
  std::vector<beam_defects::Corrosion> corrosion_vector =
      beam_defects::GetCorrosion(cloud, threshold, tolerance, min_points,
                                 max_points);
  REQUIRE(corrosion_vector[0].GetSize() == Approx(0.1));
  REQUIRE(corrosion_vector[0].GetOSIMSeverity() ==
          beam_defects::DefectOSIMSeverity::SEVERE);
  REQUIRE(corrosion_vector[0].GetMaxDim2D() == Approx(1.00499));
  REQUIRE(corrosion_vector[1].GetSize() == Approx(0.0425));
  REQUIRE(corrosion_vector[1].GetOSIMSeverity() ==
          beam_defects::DefectOSIMSeverity::MEDIUM);
  REQUIRE(corrosion_vector[1].GetMaxDim2D() == Approx(0.452769));
}
