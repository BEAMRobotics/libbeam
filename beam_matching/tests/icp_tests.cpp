#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include <pcl/io/pcd_io.h>
#include <random>

#include "beam_matching/IcpMatcher.hpp"

namespace beam_matching {

std::string scan_path;
std::string config_path;
std::string test_path = __FILE__;
std::string current_file = "icp_tests.cpp";
float threshold = 0.1;

IcpMatcher matcher;

void FileSetUp() {
  test_path.erase(test_path.end() - current_file.size(), test_path.end());
  scan_path = test_path + "data/testscan.pcd";
  config_path = test_path + "config/icp_config.json";
}

void SetUp(const IcpMatcherParams params, const Eigen::Affine3d perturb) {
  matcher.SetParams(params);

  pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile(scan_path, *test_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_test_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*test_cloud, *transformed_test_cloud, perturb);

  matcher.Setup(test_cloud, transformed_test_cloud);
}

TEST_CASE("Test initialization") {
  auto params = matcher.GetParams();
  REQUIRE(params.max_corr == 3);
  REQUIRE(params.max_iter == 100);
  REQUIRE(params.t_eps == 1e-8);
  REQUIRE(params.fit_eps == 1e-2);
  REQUIRE(params.lidar_ang_covar == 7.78e-9);
  REQUIRE(params.lidar_lin_covar == 2.5e-4);
  REQUIRE(params.multiscale_steps == 3);
  REQUIRE(params.res == 0.1f);
}

TEST_CASE("Test zero displacement case without downsampling") {
  FileSetUp();
  Eigen::Affine3d perturb;
  Eigen::Affine3d result;
  bool match_success = false;

  // setup
  perturb = Eigen::Affine3d::Identity();
  perturb.translation() << 0, 0, 0;
  IcpMatcherParams params(config_path);
  params.res = -1;

  SetUp(params, perturb);

  // test and assert
  match_success = matcher.Match();
  double diff = (matcher.GetResult().matrix() - perturb.matrix()).norm();
  REQUIRE(match_success == true);
  REQUIRE(diff < threshold);
}

TEST_CASE("Test zero displacement case using voxel downsampling") {
  Eigen::Affine3d perturb;
  Eigen::Affine3d result;
  bool match_success = false;

  // setup
  perturb = Eigen::Affine3d::Identity();
  perturb.translation() << 0, 0, 0;
  IcpMatcherParams params(config_path);
  params.res = 0.05f;

  SetUp(params, perturb);

  // test and assert
  match_success = matcher.Match();
  double diff = (matcher.GetResult().matrix() - perturb.matrix()).norm();
  REQUIRE(match_success == true);
  REQUIRE(diff < threshold);
}

TEST_CASE("Test mall displacement case using voxel downsampling") {
  Eigen::Affine3d perturb;
  Eigen::Affine3d result;
  bool match_success = false;

  // setup
  perturb = Eigen::Affine3d::Identity();
  perturb.translation() << 0.2, 0, 0;
  IcpMatcherParams params(config_path);
  params.res = 0.05f;

  SetUp(params, perturb);

  // test and assert
  match_success = matcher.Match();
  double diff = (matcher.GetResult().matrix() - perturb.matrix()).norm();
  REQUIRE(match_success == true);
  REQUIRE(diff < threshold);
}

TEST_CASE("Test small information case using voxel downsampling") {
  Eigen::Affine3d perturb;
  Eigen::Affine3d result;
  bool match_success = false;

  // setup
  perturb = Eigen::Affine3d::Identity();
  perturb.translation() << 0.2, 0, 0;
  IcpMatcherParams params(config_path);
  params.res = 0.05f;

  SetUp(params, perturb);

  // test and assert
  match_success = matcher.Match();
  matcher.EstimateInfo();
  auto info = matcher.GetInfo();
  double diff = (matcher.GetResult().matrix() - perturb.matrix()).norm();
  REQUIRE(match_success == true);
  REQUIRE(info(0, 0) > 0);
  REQUIRE(diff < threshold);
}

TEST_CASE("Test small displacement case using voxel downsampling and "
          "multiscale matching") {
  Eigen::Affine3d perturb;
  Eigen::Affine3d result;
  bool match_success = false;

  // setup
  perturb = Eigen::Affine3d::Identity();
  perturb.translation() << 0.2, 0, 0;
  IcpMatcherParams params(config_path);
  params.res = 0.1f;
  params.multiscale_steps = 3;

  SetUp(params, perturb);

  // test and assert
  match_success = matcher.Match();
  matcher.EstimateInfo();
  double diff = (matcher.GetResult().matrix() - perturb.matrix()).norm();
  REQUIRE(match_success == true);
  REQUIRE(diff < threshold);
}

// Small information using voxel downsampling
TEST_CASE("Test small information case using voxel downsampling and different "
          "covariance methods") {
  pcl::PointCloud<pcl::PointXYZ>::Ptr ref(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target(
      new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile(scan_path, *ref);
  Eigen::Affine3d perturb;
  Eigen::Affine3d result;
  double lower_bound = -0.3;
  double upper_bound = 0.3;
  std::uniform_real_distribution<double> unif(lower_bound, upper_bound);
  std::default_random_engine re;
  perturb = Eigen::Affine3d::Identity();
  perturb.translation() << 0.2, 0, 0;

  pcl::transformPointCloud(*ref, *target, perturb);
  // need to distort one scan or there will be infinite information
  for (size_t i = 0; i < target->size(); i++) {
    target->at(i).x += unif(re);
    target->at(i).y += unif(re);
    target->at(i).z += unif(re);
  }

  IcpMatcherParams params(config_path);
  params.res = 0.05f;
  params.covar_estimator = IcpMatcherParams::covar_method::LUMold;
  IcpMatcher matcher1(params);

  matcher1.Setup(ref, target);
  matcher1.Match();
  matcher1.EstimateInfo();
  auto info1 = matcher1.GetInfo();
  Eigen::Matrix<double, 6, 6> expected_info1;
  expected_info1 <<  78.5015,        0,        0,        0, -80.0395, -45.9599,
                           0,  78.5015,        0,  45.9599, -16.7882,        0,
                           0,        0,  78.5015,  80.0395,        0,  16.7882,
                           0,  45.9599,  80.0395,  1740.08,  37.3178,  527.515,
                    -80.0395, -16.7882,        0,  37.3178,  9518.46, -9.33379,
                    -45.9599,        0,  16.7882,  527.515, -9.33379,  8061.41;

  params.covar_estimator = IcpMatcherParams::covar_method::LUM;
  IcpMatcher matcher2(params);
  matcher2.Setup(ref, target);
  matcher2.Match();
  matcher2.EstimateInfo();
  auto info2 = matcher2.GetInfo();
  Eigen::Matrix<double, 6, 6> expected_info2;
  expected_info2 << 78.0082,        0,        0,        0,  -79.588, -45.6311,
                          0,  78.0082,        0,  45.6311, -16.9183,        0,
                          0,        0,  78.0082,   79.588,        0,  16.9183,
                          0,  45.6311,   79.588,  1729.19,   36.857,  524.577,
                    -79.588, -16.9183,        0,   36.857,  9458.81, -9.24851,
                   -45.6311,        0,  16.9183,  524.577, -9.24851,  8010.75;

  double diff1 = (info1 - expected_info1).norm();
  double diff2 = (info2 - expected_info2).norm();
  REQUIRE(info1(0, 0) > 0);
  REQUIRE(info2(0, 0) > 0);
  REQUIRE(diff1 < 0.01);
  REQUIRE(diff2 < 0.01);
}

} // namespace beam_matching
