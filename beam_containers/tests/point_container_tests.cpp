#define CATCH_CONFIG_MAIN
#include "beam_containers/PointBridge.h"
#include <Eigen/Geometry>
#include <catch2/catch.hpp>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>

TEST_CASE("Testing building PointBridge object from scratch") {
  beam_containers::PointBridge point;
  float x = 0, y = 0, z = 0, intensity = 1.0, thermal = 1, crack = 1.0,
        spall = 1.0, corrosion = 1.0, delam = 1.0;
  int r = 255, g = 255, b = 255;

  point.x = x;
  point.y = y;
  point.z = z;
  point.r = r;
  point.g = g;
  point.b = b;
  point.intensity = intensity;
  point.thermal = thermal;
  point.crack = crack;
  point.spall = spall;
  point.corrosion = corrosion;
  point.delam = delam;

  REQUIRE(point.x == x);
  REQUIRE(point.y == y);
  REQUIRE(point.z == z);
  REQUIRE(point.r == r);
  REQUIRE(point.g == g);
  REQUIRE(point.b == b);
  REQUIRE(point.intensity == intensity);
  REQUIRE(point.thermal == thermal);
  REQUIRE(point.crack == crack);
  REQUIRE(point.spall == spall);
  REQUIRE(corrosion == corrosion);
  REQUIRE(point.delam == delam);
}

TEST_CASE("Test creating point cloud of point type PointBridge and perform "
          "regular PCL operations") {
  pcl::PointCloud<beam_containers::PointBridge>::Ptr point_cloud1(
      new pcl::PointCloud<beam_containers::PointBridge>);
  pcl::PointCloud<beam_containers::PointBridge>::Ptr point_cloud2(
      new pcl::PointCloud<beam_containers::PointBridge>);

  Eigen::Affine3d transformation;
  Eigen::Matrix4d transformation_matrix;
  Eigen::Vector3d translation_vector(1, 0, 0);
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  transformation_matrix.setIdentity();
  transformation_matrix.block<3, 3>(0, 0) = rotation_matrix;
  transformation_matrix.block<3, 1>(0, 3) = translation_vector;

  transformation.matrix() = transformation_matrix;

  float y = 0, z = 0, intensity = 1.0, thermal = 0, crack = 1.0, spall = 0.0,
        corrosion = 0.0, delam = 0.0;
  int r = 0, g = 0, b = 0;

  beam_containers::PointBridge point;
  for (int x = 0; x < 10; x++) {
    point.x = x;
    point.y = y;
    point.z = z;
    point.r = r;
    point.g = g;
    point.b = b;
    point.intensity = intensity;
    point.thermal = thermal;
    point.crack = crack;
    point.spall = spall;
    point.corrosion = corrosion;
    point.delam = delam;
    point_cloud1->push_back(point);
  }

  bool did_transform_correctly = true;
  pcl::transformPointCloud(*point_cloud1, *point_cloud2, transformation);
  for (uint32_t i = 0; i < point_cloud1->points.size(); i++) {
    if (pcl::euclideanDistance(point_cloud1->points[i],
                               point_cloud2->points[i]) != 1) {
      did_transform_correctly = false;
    }
  }

  REQUIRE(did_transform_correctly == true);
}
