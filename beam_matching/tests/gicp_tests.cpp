#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/search/impl/search.hpp>

#include <beam_matching/GicpMatcher.hpp>

namespace beam_matching {

std::string scan_path;
std::string config_path;
std::string test_path = __FILE__;
std::string current_file = "gicp_tests.cpp";

const auto TEST_SCAN = "tests/data/testscan.pcd";
const auto TEST_CONFIG = "tests/config/gicp.yaml";

GicpMatcher matcher;

void FileSetup() {
  test_path.erase(test_path.end() - current_file.size(), test_path.end());
  scan_path = test_path + "data/testscan.pcd";
  config_path = test_path + "config/gicp_config.json";
}

void SetUp(const GicpMatcherParams params, const Eigen::Affine3d perturb) {
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
  REQUIRE(params.corr_rand == 10);
  REQUIRE(params.max_iter == 100);
  REQUIRE(params.r_eps == 1e-8);
  REQUIRE(params.fit_eps == 1e-2);
  REQUIRE(params.res == 0.1f);
}

// Zero displacement without downsampling
TEST_CASE("Test zero displacement without downsampling") {
  FileSetup();

  Eigen::Affine3d perturb;
  Eigen::Affine3d result;
  bool match_success = false;

  // setup
  perturb = Eigen::Affine3d::Identity();
  perturb.translation() << 0, 0, 0;
  GicpMatcherParams params(config_path);
  params.res = -1;
  SetUp(params, perturb);

  // test and assert
  match_success = matcher.Match();
  double diff = (matcher.GetResult().matrix() - perturb.matrix()).norm();
  REQUIRE(match_success == true);
  REQUIRE(diff < 0.1);
}

TEST_CASE("Test zero displacement using voxel downsampling") {
  Eigen::Affine3d perturb;
  Eigen::Affine3d result;
  bool match_success = false;

  // setup
  perturb = Eigen::Affine3d::Identity();
  perturb.translation() << 0, 0, 0;
  GicpMatcherParams params(config_path);
  params.res = 0.05f;
  SetUp(params, perturb);

  // test and assert
  match_success = matcher.Match();
  double diff = (matcher.GetResult().matrix() - perturb.matrix()).norm();
  REQUIRE(match_success == true);
  REQUIRE(diff < 0.1);
}

TEST_CASE("Test small displacement using voxel downsampling") {
  Eigen::Affine3d perturb;
  Eigen::Affine3d result;
  bool match_success = false;

  // setup
  perturb = Eigen::Affine3d::Identity();
  perturb.translation() << 0.2, 0, 0;
  GicpMatcherParams params(config_path);
  params.res = 0.05f;
  SetUp(params, perturb);

  // test and assert
  match_success = matcher.Match();
  double diff = (matcher.GetResult().matrix() - perturb.matrix()).norm();
  REQUIRE(match_success == true);
  REQUIRE(diff < 0.1);
}

}  // namespace beam_matching
