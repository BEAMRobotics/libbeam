#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/search/impl/search.hpp>

#include <beam_matching/NdtMatcher.h>

namespace beam_matching {

std::string scan_path;
std::string config_path;
std::string test_path = __FILE__;
std::string current_file = "ndt_tests.cpp";
const float threshold = 0.12;

NdtMatcher matcher;

void FileSetUp() {
  test_path.erase(test_path.end() - current_file.size(), test_path.end());
  scan_path = test_path + "data/testscan.pcd";
  config_path = test_path + "config/ndt_config.json";
}

void SetUp(const NdtMatcherParams params, const Eigen::Affine3d perturb) {
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
  REQUIRE(params.step_size == 3);
  REQUIRE(params.max_iter == 100);
  REQUIRE(params.t_eps == 1e-8);
  REQUIRE(params.res == 5);
}

// Zero displacement using resolution from config
TEST_CASE("Zero displacement using resolution from config") {
  FileSetUp();

  Eigen::Affine3d perturb;
  Eigen::Affine3d result;
  bool match_success = false;

  // setup
  perturb = Eigen::Affine3d::Identity();
  perturb.translation() << 0, 0, 0;
  NdtMatcherParams params(config_path);
  SetUp(params, perturb);

  // test and assert
  match_success = matcher.Match();
  double diff = (matcher.GetResult().matrix() - perturb.matrix()).norm();
  REQUIRE(match_success == true);
  REQUIRE(diff < threshold);
}

// Zero displacement using resolution set by constructor
TEST_CASE("Zero displacement using resolution set by constructor") {
  Eigen::Affine3d perturb;
  Eigen::Affine3d result;
  bool match_success = false;

  // setup
  perturb = Eigen::Affine3d::Identity();
  perturb.translation() << 0, 0, 0;

  NdtMatcherParams params(config_path);
  params.res = 0.1f;
  SetUp(params, perturb);

  // test and assert
  match_success = matcher.Match();
  double diff = (matcher.GetResult().matrix() - perturb.matrix()).norm();
  REQUIRE(match_success == true);
  REQUIRE(diff < threshold);
}

// Small displacement using resolution set by constructor
TEST_CASE("Small displacement using resolution set by constructor") {
  Eigen::Affine3d perturb;
  Eigen::Affine3d result;
  bool match_success = false;

  // setup
  perturb = Eigen::Affine3d::Identity();
  perturb.translation() << 0.2, 0, 0;
  NdtMatcherParams params(config_path);
  params.res = 0.3f;
  SetUp(params, perturb);

  // test and assert
  match_success = matcher.Match();
  double diff = (matcher.GetResult().matrix() - perturb.matrix()).norm();
  REQUIRE(match_success == true);
  REQUIRE(diff < threshold);
}

} // namespace beam_matching
