#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include <pcl/io/pcd_io.h>
#include <random>

#include "beam_matching/IcpMatcher.hpp"

namespace beam_matching {

const auto TEST_SCAN = "tests/data/testscan.pcd";
const auto TEST_CONFIG = "tests/config/icp.yaml";

IcpMatcher matcher;

void SetUp(const IcpMatcherParams params, const Eigen::Affine3d perturb) {
  matcher = new IcpMatcher(params);

  matcher.setRef(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>());
  matcher.setTarget(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>());
  pcl::io::loadPCDFile(TEST_SCAE, *(matcher.ref));

  pcl::transformPointCloud(*(matcher.ref), *(matcher.target), perturb);
  matcher.setup(matcher.ref, matcher.target);
}

TEST(ICPTests, initialization) {
    IcpMatcher matcher(IcpMatcherParams());
}

// Zero displacement without downsampling
TEST_F(ICPTest, fullResNullMatch) {
    Affine3 perturb;
    Affine3 result;
    bool match_success = false;

    // setup
    perturb = Affine3::Identity();
    perturb.translation() << 0, 0, 0;
    IcpMatcherParams params(TEST_CONFIG);
    params.res = -1;
    this->initMatcher(params, perturb);

    // test and assert
    match_success = matcher->match();
    double diff = (matcher->getResult().matrix() - perturb.matrix()).norm();
    EXPECT_TRUE(match_success);
    EXPECT_LT(diff, this->threshold);
}

// Zero displacement using voxel downsampling
TEST_F(ICPTest, nullDisplacement) {
    Affine3 perturb;
    Affine3 result;
    bool match_success = false;

    // setup
    perturb = Affine3::Identity();
    perturb.translation() << 0, 0, 0;
    IcpMatcherParams params(TEST_CONFIG);
    params.res = 0.05f;
    this->initMatcher(params, perturb);

    // test and assert
    match_success = matcher->match();
    double diff = (matcher->getResult().matrix() - perturb.matrix()).norm();
    EXPECT_TRUE(match_success);
    EXPECT_LT(diff, this->threshold);
}

// Small displacement using voxel downsampling
TEST_F(ICPTest, smallDisplacement) {
    Affine3 perturb;
    Affine3 result;
    bool match_success = false;

    // setup
    perturb = Affine3::Identity();
    perturb.translation() << 0.2, 0, 0;
    IcpMatcherParams params(TEST_CONFIG);
    params.res = 0.05f;
    this->initMatcher(params, perturb);

    // test and assert
    match_success = matcher->match();
    double diff = (matcher->getResult().matrix() - perturb.matrix()).norm();
    EXPECT_TRUE(match_success);
    EXPECT_LT(diff, this->threshold);
}

// Small information using voxel downsampling
TEST_F(ICPTest, smallinfo) {
    Affine3 perturb;
    Affine3 result;
    bool match_success = false;

    // setup
    perturb = Affine3::Identity();
    perturb.translation() << 0.2, 0, 0;
    IcpMatcherParams params(TEST_CONFIG);
    params.res = 0.05f;
    this->initMatcher(params, perturb);

    // test and assert
    match_success = matcher->match();
    matcher->estimateInfo();
    auto info = matcher->getInfo();
    double diff = (matcher->getResult().matrix() - perturb.matrix()).norm();
    EXPECT_TRUE(match_success);
    EXPECT_GT(info(0, 0), 0);
    EXPECT_LT(diff, this->threshold);
}

// Small displacement using voxel downsampling
// and multiscale matching
TEST_F(ICPTest, multiscale) {
    Affine3 perturb;
    Affine3 result;
    bool match_success = false;

    // setup
    perturb = Affine3::Identity();
    perturb.translation() << 0.2, 0, 0;
    IcpMatcherParams params(TEST_CONFIG);
    params.res = 0.1f;
    params.multiscale_steps = 3;
    this->initMatcher(params, perturb);

    // test and assert
    match_success = matcher->match();
    matcher->estimateInfo();
    double diff = (matcher->getResult().matrix() - perturb.matrix()).norm();
    EXPECT_TRUE(match_success);
    EXPECT_LT(diff, this->threshold);
}

// Small information using voxel downsampling
TEST(ICPTests, lumvslum) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr ref, target;
    ref = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    target = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::io::loadPCDFile(TEST_SCAN, *(ref));
    Affine3 perturb;
    Affine3 result;
    double lower_bound = -0.3;
    double upper_bound = 0.3;
    std::uniform_real_distribution<double> unif(lower_bound, upper_bound);
    std::default_random_engine re;
    perturb = Affine3::Identity();
    perturb.translation() << 0.2, 0, 0;

    pcl::transformPointCloud(*(ref), *(target), perturb);
    // need to distort one scan or there will be infinite information
    for (size_t i = 0; i < target->size(); i++) {
        target->at(i).x += unif(re);
        target->at(i).y += unif(re);
        target->at(i).z += unif(re);
    }


    IcpMatcherParams params(TEST_CONFIG);
    params.res = 0.05f;
    params.covar_estimator = IcpMatcherParams::covar_method::LUMold;
    IcpMatcher matcher1(params);


    matcher1.setup(ref, target);
    matcher1.match();
    matcher1.estimateInfo();
    auto info1 = matcher1.getInfo();

    params.covar_estimator = IcpMatcherParams::covar_method::LUM;
    IcpMatcher matcher2(params);
    matcher2.setup(ref, target);
    matcher2.match();
    matcher2.estimateInfo();
    auto info2 = matcher2.getInfo();

    double diff = (info1 - info2).norm();
    EXPECT_GT(info1(0, 0), 0);
    EXPECT_LT(diff, 0.01);
}

}  // end of namespace wave
