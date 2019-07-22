#define CATCH_CONFIG_MAIN
#include "beam_filtering/CropBox.h"
#include "beam_utils/math.hpp"
#include <boost/filesystem.hpp>
#include <catch2/catch.hpp>
#include <pcl/common/transforms.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

TEST_CASE("Test invalid inputs and params i/o") {
  PointCloud::Ptr input_cloud = boost::make_shared<PointCloud>();
  PointCloud::Ptr output_cloud = boost::make_shared<PointCloud>();
  beam_filtering::CropBox cropper;
  REQUIRE_THROWS(cropper.Filter(*input_cloud, *output_cloud));
  Eigen::Vector3f min_vec(-1, -1, -1);
  Eigen::Vector3f max_vec(1, 1, 1);
  cropper.SetMinVector(min_vec);
  REQUIRE_THROWS(cropper.Filter(*input_cloud, *output_cloud));
  cropper.SetMaxVector(max_vec);
  REQUIRE(cropper.GetMinVector() == min_vec);
  REQUIRE(cropper.GetMaxVector() == max_vec);
  REQUIRE_NOTHROW(cropper.Filter(*input_cloud, *output_cloud));
}

// function to return random double between -1 and 1 of precision "precision"
double GetRandNum(int precision) {
  int total_options = precision * 2 + 1;
  int randInt = rand() % total_options + (-precision);
  double randDouble = double(randInt);
  return randDouble / precision;
}

TEST_CASE("Test cropper is working on real points") {
  PointCloud::Ptr input_cloud = boost::make_shared<PointCloud>();
  PointCloud::Ptr output_cloud = boost::make_shared<PointCloud>();
  beam_filtering::CropBox cropper;
  Eigen::Vector3f min_vec(-1, -1, -1);
  Eigen::Vector3f max_vec(1, 1, 1);

  // create point cloud with 50 random points within cropbox
  for (size_t i = 0; i < 50; ++i) {
    pcl::PointXYZ point;
    point.x = GetRandNum(1000);
    point.y = GetRandNum(1000);
    point.z = GetRandNum(1000);
    input_cloud->points.push_back(point);
  }
  // create point cloud with 100 random points outside cropbox
  for (size_t i = 0; i < 100; ++i) {
    bool point_invalid = true;
    while (point_invalid) {
      pcl::PointXYZ point;
      point.x = 100 * GetRandNum(1000);
      point.y = 100 * GetRandNum(1000);
      point.z = 100 * GetRandNum(1000);
      if (point.x < min_vec[0] || point.y < min_vec[1] ||
          point.z < min_vec[2] || point.x > max_vec[0] ||
          point.y > max_vec[1] || point.z > max_vec[2]) {
        point_invalid = false;
        input_cloud->points.push_back(point);
      }
    }
  }

  cropper.SetMinVector(min_vec);
  cropper.SetMaxVector(max_vec);
  cropper.Filter(*input_cloud, *output_cloud);
  REQUIRE(input_cloud->points.size() == 150);
  REQUIRE(output_cloud->points.size() == 50);

  // test transformation of cropbox
  Eigen::Affine3f T_box_cloud;
  Eigen::Vector3f t_box_cloud(1, 2, 3);
  Eigen::Matrix3f R_box_cloud;
  R_box_cloud = Eigen::AngleAxisf(30, Eigen::Vector3f::UnitZ());
  T_box_cloud.matrix().block(0, 3, 3, 1) = t_box_cloud;
  T_box_cloud.matrix().block(0, 0, 3, 3) = R_box_cloud;

  PointCloud::Ptr transformed_cloud(new PointCloud);
  pcl::transformPointCloud(*input_cloud, *transformed_cloud,
                           T_box_cloud.inverse());

  cropper.SetTransform(T_box_cloud);
  cropper.Filter(*transformed_cloud, *output_cloud);
  REQUIRE(transformed_cloud->points.size() == 150);
  REQUIRE(output_cloud->points.size() == 50);

  // test removing inside points
    beam_filtering::CropBox cropper2;
    cropper2.SetMinVector(min_vec);
    cropper2.SetMaxVector(max_vec);
    cropper2.SetRemoveOutsidePoints(false);
    PointCloud::Ptr output_cloud2 = boost::make_shared<PointCloud>();
    cropper2.Filter(*input_cloud, *output_cloud2);
    REQUIRE(cropper2.GetRemoveOutsidePoints() == false);
    REQUIRE(output_cloud2->points.size() == 100);
}
