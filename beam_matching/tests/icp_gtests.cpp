#include <gtest/gtest.h>

#include <random>

#include <pcl/io/pcd_io.h>
#include <pcl/search/impl/search.hpp>

#include <beam_matching/IcpMatcher.h>
#include <beam_utils/math.h>
#include <beam_utils/se3.h>
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
  params.covar_estimator = IcpMatcherParams::CovarMethod::CENSI;
  IcpMatcher matcher1(params);

  matcher1.Setup(ref, target);
  matcher1.Match();
  auto cov1 = matcher1.GetCovariance();
  EXPECT_TRUE(!cov1.isIdentity());

  params.covar_estimator = IcpMatcherParams::CovarMethod::LUM;
  IcpMatcher matcher2(params);
  matcher2.Setup(ref, target);
  matcher2.Match();
  auto cov2 = matcher2.GetCovariance();
  EXPECT_TRUE(!cov2.isIdentity());
}

} // namespace beam_matching
