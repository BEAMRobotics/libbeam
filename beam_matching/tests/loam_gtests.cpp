#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>

#include <beam_matching/LoamMatcher.h>
#include <beam_matching/loam/LoamFeatureExtractor.h>
#include <beam_matching/loam/LoamParams.h>
#include <beam_matching/loam/LoamPointCloud.h>
#include <beam_matching/loam/LoamScanRegistration.h>
#include <beam_utils/log.h>
#include <beam_utils/math.h>
#include <beam_utils/se3.h>
#include <beam_utils/pointclouds.h>

namespace beam_matching {

class Data {
public:
  Data() {
    // get file paths
    std::string test_path = __FILE__;
    std::string current_file = "loam_gtests.cpp";
    test_path.erase(test_path.end() - current_file.size(), test_path.end());
    std::string scan_path = test_path + "data/test_scan_vlp16.pcd";
    std::string config_path = test_path + "config/loam_config.json";

    // load matcher params
    params = std::make_shared<LoamParams>(config_path);

    // create poses
    // srand(time(NULL));
    double max_pert_rot{10};
    double max_pert_trans{0.05};

    // set first cloud pose to identity
    T_WORLD_CLOUD1 = Eigen::Matrix4d::Identity();

    // create larger perturbation for cloud 2
    Eigen::VectorXd perturb(6);
    perturb << beam::randf(max_pert_rot, -max_pert_rot),
        beam::randf(max_pert_rot, -max_pert_rot),
        beam::randf(max_pert_rot, -max_pert_rot),
        beam::randf(max_pert_trans, -max_pert_trans),
        beam::randf(max_pert_trans, -max_pert_trans),
        beam::randf(max_pert_trans, -max_pert_trans);
    T_WORLD_CLOUD2 = beam::PerturbTransformDegM(T_WORLD_CLOUD1, perturb);
    T_CLOUD2_CLOUD1 = beam::InvertTransform(T_WORLD_CLOUD2) * T_WORLD_CLOUD1;

    // create small perturbation for cloud 3
    Eigen::VectorXd perturb_small(6);
    perturb_small << 0.5, -0.5, 3, beam::randf(max_pert_trans, -max_pert_trans),
        beam::randf(max_pert_trans, -max_pert_trans),
        beam::randf(max_pert_trans, -max_pert_trans);
    T_WORLD_CLOUD3 = beam::PerturbTransformDegM(T_WORLD_CLOUD1, perturb_small);
    T_CLOUD3_CLOUD1 = beam::InvertTransform(T_WORLD_CLOUD3) * T_WORLD_CLOUD1;

    // create clouds
    cloud1 = std::make_shared<PointCloud>();
    pcl::io::loadPCDFile(scan_path, *cloud1);
    cloud2 = std::make_shared<PointCloud>();
    pcl::transformPointCloud(*cloud1, *cloud2, T_CLOUD2_CLOUD1);
    cloud3 = std::make_shared<PointCloud>();
    pcl::transformPointCloud(*cloud1, *cloud3, T_CLOUD3_CLOUD1);
  }

  Eigen::Matrix4d T_WORLD_CLOUD1;
  Eigen::Matrix4d T_WORLD_CLOUD2;
  Eigen::Matrix4d T_WORLD_CLOUD3;
  Eigen::Matrix4d T_CLOUD2_CLOUD1;
  Eigen::Matrix4d T_CLOUD3_CLOUD1;
  PointCloudPtr cloud1;
  PointCloudPtr cloud2;
  PointCloudPtr cloud3;
  LoamParamsPtr params;
};

Data data_;

TEST(LoamParams, AngleBins) {
  // Test beam anle bins
  LoamParams params;
  params.number_of_beams = 3;
  params.fov_deg = 30;
  std::vector<double> bins = params.GetBeamAngleBinsDeg();
  uint8_t expected_size = 2;
  EXPECT_EQ(bins.size(), expected_size);
  EXPECT_EQ(bins[0], 7.5);
  EXPECT_EQ(bins[1], -7.5);
}

TEST(LoamFeatureExtractor, Features) {
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
  LoamPointCloud loam_cloud = fea_extractor.ExtractFeatures(*data_.cloud1);
  EXPECT_TRUE(loam_cloud.surfaces.strong.cloud.size() > 10);
  EXPECT_TRUE(loam_cloud.surfaces.weak.cloud.size() > 10);
  EXPECT_TRUE(loam_cloud.edges.strong.cloud.size() > 10);
  EXPECT_TRUE(loam_cloud.edges.weak.cloud.size() > 10);
  // loam_cloud.Save("/home/nick/tmp/loam_tests/");
}

TEST(ScanRegistration, InitialGuess) {
  LoamFeatureExtractor fea_extractor(data_.params);
  LoamScanRegistration scan_reg(data_.params);

  auto loam_cloud1 = std::make_shared<LoamPointCloud>(
      fea_extractor.ExtractFeatures(*data_.cloud1).Copy());
  auto loam_cloud2 = std::make_shared<LoamPointCloud>(
      fea_extractor.ExtractFeatures(*data_.cloud2).Copy());

  Eigen::Matrix4d T_CLOUD1_CLOUD2 =
      beam::InvertTransform(data_.T_CLOUD2_CLOUD1);
  bool reg_successful =
      scan_reg.RegisterScans(loam_cloud1, loam_cloud2, T_CLOUD1_CLOUD2);
  Eigen::Matrix4d T_CLOUD1_CLOUD2_mea = scan_reg.GetT_REF_TGT();

  ///////// save resuls ////////
  // std::string save_path = "/home/nick/tmp/loam_tests/";
  // loam_cloud1->Save(save_path + "cloud_orig/");
  // loam_cloud2->Save(save_path + "cloud_pert/");
  // loam_cloud2->TransformPointCloud(T_CLOUD1_CLOUD2_mea);
  // loam_cloud2->Save(save_path + "cloud_aligned/");
  ///////////////////////////////

  EXPECT_TRUE(reg_successful);
  EXPECT_TRUE(
      beam::ArePosesEqual(T_CLOUD1_CLOUD2_mea, T_CLOUD1_CLOUD2, 1, 0.03));
}

TEST(ScanRegistration, SmallPerturb) {
  LoamParamsPtr params = std::make_shared<LoamParams>();
  *params = *data_.params;
  params->iterate_correspondences = true;
  LoamFeatureExtractor fea_extractor(params);
  LoamScanRegistration scan_reg(params);

  auto loam_cloud1 = std::make_shared<LoamPointCloud>(
      fea_extractor.ExtractFeatures(*data_.cloud1));
  auto loam_cloud3 = std::make_shared<LoamPointCloud>(
      fea_extractor.ExtractFeatures(*data_.cloud3));

  bool reg_successful = scan_reg.RegisterScans(loam_cloud1, loam_cloud3);
  Eigen::Matrix4d T_CLOUD1_CLOUD3_mea = scan_reg.GetT_REF_TGT();
  Eigen::Matrix4d T_CLOUD1_CLOUD3 =
      beam::InvertTransform(data_.T_CLOUD3_CLOUD1);

  ///////// save resuls ////////
  // std::string save_path = "/home/nick/tmp/loam_tests/";
  // loam_cloud1->Save(save_path + "cloud_orig/");
  // loam_cloud3->Save(save_path + "cloud_pert/");
  // loam_cloud3->TransformPointCloud(T_CLOUD1_CLOUD3_mea);
  // loam_cloud3->Save(save_path + "cloud_aligned/");
  ///////////////////////////////

  EXPECT_TRUE(reg_successful);
  EXPECT_TRUE(
      beam::ArePosesEqual(T_CLOUD1_CLOUD3_mea, T_CLOUD1_CLOUD3, 1, 0.05));
}

TEST(LoamMatcher, SmallPerturb) {
  // get loam params
  LoamParamsPtr params = std::make_shared<LoamParams>();
  *params = *data_.params;
  params->iterate_correspondences = true;

  // get loam cloud
  LoamFeatureExtractor fea_extractor(params);
  auto loam_cloud1 = std::make_shared<LoamPointCloud>(
      fea_extractor.ExtractFeatures(*data_.cloud1));
  auto loam_cloud3 = std::make_shared<LoamPointCloud>(
      fea_extractor.ExtractFeatures(*data_.cloud3));

  // setup matcher
  LoamMatcher matcher(*params);
  matcher.SetRef(loam_cloud1);
  matcher.SetTarget(loam_cloud3);

  // match
  bool match_success = matcher.Match();
  Eigen::Matrix4d T_CLOUD3_CLOUD1_meas = matcher.GetResult().matrix();

  EXPECT_TRUE(match_success);
  EXPECT_TRUE(beam::ArePosesEqual(T_CLOUD3_CLOUD1_meas, data_.T_CLOUD3_CLOUD1,
                                  1, 0.05));
}

TEST(LoamMatcher, LargePerturb) {
  // get loam params
  LoamParamsPtr params = std::make_shared<LoamParams>();
  *params = *data_.params;
  params->iterate_correspondences = true;

  // get loam cloud
  LoamFeatureExtractor fea_extractor(params);
  auto loam_cloud1 = std::make_shared<LoamPointCloud>(
      fea_extractor.ExtractFeatures(*data_.cloud1));
  auto loam_cloud2 = std::make_shared<LoamPointCloud>(
      fea_extractor.ExtractFeatures(*data_.cloud2));

  // setup matcher
  LoamMatcher matcher(*params);
  matcher.SetRef(loam_cloud1);
  matcher.SetTarget(loam_cloud2);

  // match
  bool match_success = matcher.Match();
  Eigen::Matrix4d T_CLOUD2_CLOUD1_meas = matcher.GetResult().matrix();

  EXPECT_TRUE(match_success);
  EXPECT_TRUE(beam::ArePosesEqual(T_CLOUD2_CLOUD1_meas, data_.T_CLOUD2_CLOUD1,
                                  1, 0.05));
}

} // namespace beam_matching
