#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>
#include <pcl/io/pcd_io.h>

#include <beam_matching/LoamMatcher.h>
#include <beam_matching/loam/LoamPointCloud.h>
#include <beam_matching/loam/LoamParams.h>
#include <beam_matching/loam/LoamFeatureExtractor.h>
#include <beam_matching/loam/LoamScanRegistration.h>
#include <beam_utils/math.h>
#include <beam_utils/pointclouds.h>

namespace beam_matching {


class Data {
public:
  Data() {
    // get file paths
    std::string test_path = __FILE__;
    std::string current_file = "loam_tests.cpp";
    test_path.erase(test_path.end() - current_file.size(), test_path.end());
    std::string scan_path1 = test_path + "data/testscan.pcd";
    // std::string config_path = test_path + "config/loam_config.json";

    // load matcher params
    // params = LoamParams(config_path);

    // create poses
    srand(time(NULL));
    double max_pert_rot{10};
    double max_pert_trans{0.05};
    T_WORLD_CLOUD1 = Eigen::Matrix4d::Identity();
    Eigen::VectorXd perturb(6);
    perturb << beam::randf(max_pert_rot, -max_pert_rot),
        beam::randf(max_pert_rot, -max_pert_rot),
        beam::randf(max_pert_rot, -max_pert_rot),
        beam::randf(max_pert_trans, -max_pert_trans),
        beam::randf(max_pert_trans, -max_pert_trans),
        beam::randf(max_pert_trans, -max_pert_trans);
    T_WORLD_CLOUD2 = beam::PerturbTransformDegM(T_WORLD_CLOUD1, perturb);

    // lidar_scan = boost::make_shared<PointCloud>();
    // pcl::io::loadPCDFile(scan_path1, *lidar_scan);
    // lidar_scan_pert = boost::make_shared<PointCloud>();
    // pcl::transformPointCloud(*lidar_scan, *lidar_scan_pert,
    //                          beam::InvertTransform(T_WORLD_CLOUD2));

  }

  Eigen::Matrix4d T_WORLD_CLOUD1;
  Eigen::Matrix4d T_WORLD_CLOUD2;
  // LoamMatcherParams params;
  // PointCloudPtr lidar_scan;
  // PointCloudPtr lidar_scan_pert;
};

Data data_;

TEST_CASE("Test LoamParams") {
  // Test beam anle bins
  LoamParams params;
  params.number_of_beams = 4;
  params.fov_deg = 30;
  std::vector<double> bins = params.GetBeamAngleBinsDeg();
  REQUIRE(bins.size() == 4);
  REQUIRE(bins[0] == 30/2);
  REQUIRE(bins[1] == 30/2 - 30/3);
  REQUIRE(bins[2] == 30/2 - 30/3 * 2);
  REQUIRE(bins[3] == 30/2 - 30/3 * 3);
  REQUIRE(bins[3] == -30/2);
  
}

} // namespace beam_matching
