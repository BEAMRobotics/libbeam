#include <gtest/gtest.h>

#include <random>

#include <pcl/io/pcd_io.h>
#include <pcl/search/impl/search.hpp>

#include <beam_matching/IcpMatcher.h>
#include <beam_utils/math.h>
#include <beam_utils/pointclouds.h>

namespace beam_matching {

class Data {
public:
  Data() {
    // get file paths
    std::string test_path = __FILE__;
    std::string current_file = "icp_gtests.cpp";
    test_path.erase(test_path.end() - current_file.size(), test_path.end());
    std::string scan_path = test_path + "data/test_scan_vlp16.pcd";
    std::string config_path = test_path + "config/icp_config.json";

    // load matcher params
    params = IcpMatcherParams(config_path);

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
    T_CLOUD2_CLOUD1 = beam::InvertTransform(T_WORLD_CLOUD2) * T_WORLD_CLOUD1;

    cloud1 = std::make_shared<PointCloud>();
    pcl::io::loadPCDFile(scan_path, *cloud1);
    cloud2 = std::make_shared<PointCloud>();
    pcl::transformPointCloud(*cloud1, *cloud2, T_CLOUD2_CLOUD1);
  }

  Eigen::Matrix4d T_WORLD_CLOUD1;
  Eigen::Matrix4d T_WORLD_CLOUD2;
  Eigen::Matrix4d T_CLOUD2_CLOUD1;
  PointCloudPtr cloud1;
  PointCloudPtr cloud2;
  IcpMatcherParams params;
};

Data data_;

TEST(IcpMatcherParams, initialization) {
  EXPECT_TRUE(data_.params.max_corr == 3);
  EXPECT_TRUE(data_.params.max_iter == 100);
  EXPECT_TRUE(data_.params.t_eps == 1e-8);
  EXPECT_TRUE(data_.params.fit_eps == 1e-2);
  EXPECT_TRUE(data_.params.lidar_ang_covar == 7.78e-9);
  EXPECT_TRUE(data_.params.lidar_lin_covar == 2.5e-4);
  EXPECT_TRUE(data_.params.multiscale_steps == 0);
  EXPECT_TRUE(data_.params.res == 0.1f);
}

TEST(IcpMatcher, NoDownsampling) {
  // setup
  IcpMatcherParams params2 = data_.params;
  params2.res = -1;

  // test and assert
  IcpMatcher matcher;
  matcher.SetParams(params2);
  matcher.SetRef(data_.cloud1);
  matcher.SetTarget(data_.cloud2);
  bool match_success = matcher.Match();

  // check result
  Eigen::Matrix4d T_CLOUD2_CLOUD1_meas = matcher.GetResult().matrix();
  EXPECT_TRUE(match_success);
  EXPECT_TRUE(beam::ArePosesEqual(data_.T_CLOUD2_CLOUD1, T_CLOUD2_CLOUD1_meas,
                                  1, 0.05));
}

TEST(IcpMatcher, Downsampling) {
  // setup
  IcpMatcherParams params2 = data_.params;
  params2.res = 0.05f;

  // test and assert
  IcpMatcher matcher;
  matcher.SetParams(params2);
  matcher.SetRef(data_.cloud1);
  matcher.SetTarget(data_.cloud2);
  bool match_success = matcher.Match();

  // check result
  Eigen::Matrix4d T_CLOUD2_CLOUD1_meas = matcher.GetResult().matrix();
  EXPECT_TRUE(match_success);
  EXPECT_TRUE(beam::ArePosesEqual(data_.T_CLOUD2_CLOUD1, T_CLOUD2_CLOUD1_meas,
                                  1, 0.05));
}

TEST(IcpMatcher, MultiScaleDownsampling) {
  // setup
  IcpMatcherParams params2 = data_.params;
  params2.res = 0.1f;
  params2.multiscale_steps = 3;

  // test and assert
  IcpMatcher matcher;
  matcher.SetParams(params2);
  matcher.SetRef(data_.cloud1);
  matcher.SetTarget(data_.cloud2);
  bool match_success = matcher.Match();

  // check result
  Eigen::Matrix4d T_CLOUD2_CLOUD1_meas = matcher.GetResult().matrix();
  EXPECT_TRUE(match_success);
  EXPECT_TRUE(beam::ArePosesEqual(data_.T_CLOUD2_CLOUD1, T_CLOUD2_CLOUD1_meas,
                                  1, 0.05));
}

/*
TEST(IcpMatcher, Covariances) {
  PointCloud::Ptr ref = std::make_shared<PointCloud>(*data_.cloud1);
  PointCloud::Ptr target = std::make_shared<PointCloud>();
  
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

  IcpMatcherParams params = data_.params;
  params.res = 0.05f;
  params.covar_estimator = IcpMatcherParams::covar_method::LUMold;
  IcpMatcher matcher1(params);

  matcher1.Setup(ref, target);
  matcher1.Match();
  matcher1.EstimateInfo();
  auto info1 = matcher1.GetInfo();
  Eigen::Matrix<double, 6, 6> expected_info1;
  expected_info1 << 78.5015, 0, 0, 0, -80.0395, -45.9599, 0, 78.5015, 0,
      45.9599, -16.7882, 0, 0, 0, 78.5015, 80.0395, 0, 16.7882, 0, 45.9599,
      80.0395, 1740.08, 37.3178, 527.515, -80.0395, -16.7882, 0, 37.3178,
      9518.46, -9.33379, -45.9599, 0, 16.7882, 527.515, -9.33379, 8061.41;

  params.covar_estimator = IcpMatcherParams::covar_method::LUM;
  IcpMatcher matcher2(params);
  matcher2.Setup(ref, target);
  matcher2.Match();
  matcher2.EstimateInfo();
  auto info2 = matcher2.GetInfo();
  Eigen::Matrix<double, 6, 6> expected_info2;
  expected_info2 << 78.0082, 0, 0, 0, -79.588, -45.6311, 0, 78.0082, 0, 45.6311,
      -16.9183, 0, 0, 0, 78.0082, 79.588, 0, 16.9183, 0, 45.6311, 79.588,
      1729.19, 36.857, 524.577, -79.588, -16.9183, 0, 36.857, 9458.81, -9.24851,
      -45.6311, 0, 16.9183, 524.577, -9.24851, 8010.75;

  double diff1 = (info1 - expected_info1).norm();
  double diff2 = (info2 - expected_info2).norm();
  EXPECT_TRUE(info1(0, 0) > 0);
  EXPECT_TRUE(info2(0, 0) > 0);
  EXPECT_TRUE(diff1 < 0.01);
  EXPECT_TRUE(diff2 < 0.01);
}
*/

} // namespace beam_matching
