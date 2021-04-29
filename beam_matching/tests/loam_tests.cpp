#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>
#include <pcl/io/pcd_io.h>

#include <beam_matching/LoamMatcher.h>
#include <beam_matching/loam/LoamFeatureExtractor.h>
#include <beam_matching/loam/LoamParams.h>
#include <beam_matching/loam/LoamPointCloud.h>
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
    std::string scan_path1 = test_path + "data/test_scan_vlp16.pcd";
    std::string config_path = test_path + "config/loam_config.json";

    // load matcher params
    params = LoamParams(config_path);

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

    lidar_scan = std::make_shared<PointCloud>();
    pcl::io::loadPCDFile(scan_path1, *lidar_scan);
    lidar_scan_pert = std::make_shared<PointCloud>();
    pcl::transformPointCloud(*lidar_scan, *lidar_scan_pert,
                             beam::InvertTransform(T_WORLD_CLOUD2));
  }

  Eigen::Matrix4d T_WORLD_CLOUD1;
  Eigen::Matrix4d T_WORLD_CLOUD2;
  LoamParams params;
  PointCloudPtr lidar_scan;
  PointCloudPtr lidar_scan_pert;
};

Data data_;

TEST_CASE("Test LoamParams") {
  // Test beam anle bins
  LoamParams params;
  params.number_of_beams = 3;
  params.fov_deg = 30;
  std::vector<double> bins = params.GetBeamAngleBinsDeg();
  REQUIRE(bins.size() == 2);
  REQUIRE(bins[0] == 7.5);
  REQUIRE(bins[1] == -7.5);
}

TEST_CASE("Test LoamFeatureExtractor") {
  LoamParamsPtr params = std::make_shared<LoamParams>();
  params->number_of_beams = 16;
  params->fov_deg = 20;
  params->n_feature_regions = 6;
  params->curvature_region = 5;
  params->max_corner_sharp = 2;
  params->max_corner_less_sharp = 20;
  params->max_surface_flat = 4;
  params->less_flat_filter_size = 0.2;
  params->surface_curvature_threshold = 0.1;

  LoamFeatureExtractor fea_extractor(params);
  LoamPointCloud loam_cloud = fea_extractor.ExtractFeatures(*data_.lidar_scan);
  REQUIRE(loam_cloud.surfaces.strong.cloud.size() > 10);
  REQUIRE(loam_cloud.surfaces.weak.cloud.size() > 10);
  REQUIRE(loam_cloud.edges.strong.cloud.size() > 10);
  REQUIRE(loam_cloud.edges.weak.cloud.size() > 10);
  // loam_cloud.Save("/home/nick/tmp/loam_tests/");
}

TEST_CASE("Test LoamScanRegistration"){
  //
}

TEST_CASE("Test LoamMatcher"){
  //
}


} // namespace beam_matching
