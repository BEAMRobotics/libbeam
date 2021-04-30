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
    config_path = test_path + "config/loam_config.json";

    // load matcher params
    params = std::make_shared<LoamParams>(config_path);

    // create poses
    // srand(time(NULL));
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
    Eigen::VectorXd perturb_small(6);
    perturb_small << 0.5, -0.5, 3, beam::randf(max_pert_trans, -max_pert_trans),
        beam::randf(max_pert_trans, -max_pert_trans),
        beam::randf(max_pert_trans, -max_pert_trans);
    T_WORLD_CLOUD3 = beam::PerturbTransformDegM(T_WORLD_CLOUD1, perturb_small);

    lidar_scan = std::make_shared<PointCloud>();
    pcl::io::loadPCDFile(scan_path1, *lidar_scan);
    lidar_scan_pert = std::make_shared<PointCloud>();
    pcl::transformPointCloud(*lidar_scan, *lidar_scan_pert,
                             beam::InvertTransform(T_WORLD_CLOUD2));
    lidar_scan_pert_small = std::make_shared<PointCloud>();
    pcl::transformPointCloud(*lidar_scan, *lidar_scan_pert_small,
                             beam::InvertTransform(T_WORLD_CLOUD3));
  }

  std::string config_path;
  Eigen::Matrix4d T_WORLD_CLOUD1;
  Eigen::Matrix4d T_WORLD_CLOUD2;
  Eigen::Matrix4d T_WORLD_CLOUD3;
  LoamParamsPtr params;
  PointCloudPtr lidar_scan;
  PointCloudPtr lidar_scan_pert;
  PointCloudPtr lidar_scan_pert_small;
};

TEST_CASE("Test angle bins", "[LoamParams]") {
  // Test beam anle bins
  LoamParams params;
  params.number_of_beams = 3;
  params.fov_deg = 30;
  std::vector<double> bins = params.GetBeamAngleBinsDeg();
  REQUIRE(bins.size() == 2);
  REQUIRE(bins[0] == 7.5);
  REQUIRE(bins[1] == -7.5);
}

TEST_CASE("Test features are extracted for each type",
          "[LoamFeatureExtractor]") {
  Data data_;
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

TEST_CASE("Scan registration tests", "[ScanRegistration and LoamMatcher]") {
  Data data_;
  SECTION("test initial guess") {
    Eigen::Matrix4d T_gnd_truth =
        beam::InvertTransform(data_.T_WORLD_CLOUD1) * data_.T_WORLD_CLOUD2;

    LoamFeatureExtractor fea_extractor(data_.params);
    LoamScanRegistration scan_reg(data_.params);

    LoamPointCloudPtr loam_cloud = std::make_shared<LoamPointCloud>();
    LoamPointCloudPtr loam_cloud_pert = std::make_shared<LoamPointCloud>();
    *loam_cloud = fea_extractor.ExtractFeatures(*data_.lidar_scan);
    *loam_cloud_pert = fea_extractor.ExtractFeatures(*data_.lidar_scan_pert);

    bool reg_successful =
        scan_reg.RegisterScans(loam_cloud, loam_cloud_pert, T_gnd_truth);
    Eigen::Matrix4d T_meas = scan_reg.GetT_REF_TGT();

    ///////// save resuls ////////
    // std::string save_path = "/home/nick/tmp/loam_tests/";
    // loam_cloud->Save(save_path + "cloud_orig/");
    // loam_cloud_pert->Save(save_path + "cloud_pert/");
    // loam_cloud_pert->TransformPointCloud(T_meas);
    // loam_cloud_pert->Save(save_path + "cloud_aligned/");
    ///////////////////////////////

    REQUIRE(reg_successful);
    REQUIRE(beam::ArePosesEqual(T_meas, T_gnd_truth, 1, 0.03));
  }
  SECTION("test small perturb") {
    Eigen::Matrix4d T_gnd_truth =
        beam::InvertTransform(data_.T_WORLD_CLOUD1) * data_.T_WORLD_CLOUD3;

    LoamParamsPtr params = std::make_shared<LoamParams>();
    *params = *data_.params;
    params->iterate_correspondences = true;
    LoamFeatureExtractor fea_extractor(params);
    LoamScanRegistration scan_reg(params);

    LoamPointCloudPtr loam_cloud = std::make_shared<LoamPointCloud>();
    LoamPointCloudPtr loam_cloud_pert = std::make_shared<LoamPointCloud>();
    *loam_cloud = fea_extractor.ExtractFeatures(*data_.lidar_scan);
    *loam_cloud_pert =
        fea_extractor.ExtractFeatures(*data_.lidar_scan_pert_small);

    bool reg_successful = scan_reg.RegisterScans(loam_cloud, loam_cloud_pert);
    Eigen::Matrix4d T_meas = scan_reg.GetT_REF_TGT();

    ///////// save resuls ////////
    // std::string save_path = "/home/nick/tmp/loam_tests/";
    // loam_cloud->Save(save_path + "cloud_orig/");
    // loam_cloud_pert->Save(save_path + "cloud_pert/");
    // loam_cloud_pert->TransformPointCloud(T_meas);
    // loam_cloud_pert->Save(save_path + "cloud_aligned/");
    ///////////////////////////////

    REQUIRE(reg_successful);
    REQUIRE(beam::ArePosesEqual(T_meas, T_gnd_truth, 1, 0.05));
  }
  SECTION("test matcher class") {
    Eigen::Matrix4d T_gnd_truth =
        beam::InvertTransform(data_.T_WORLD_CLOUD1) * data_.T_WORLD_CLOUD3;

    LoamParams params(data_.config_path);
    params.iterate_correspondences = true;
    LoamMatcher matcher(params);

    matcher.Setup(data_.lidar_scan, data_.lidar_scan_pert_small);

    bool match_success = matcher.Match();
    Eigen::Matrix4d T_meas = matcher.GetResult().matrix();

    REQUIRE(match_success == true);
    REQUIRE(beam::ArePosesEqual(T_meas, T_gnd_truth, 1, 0.05));
  }
}

} // namespace beam_matching
