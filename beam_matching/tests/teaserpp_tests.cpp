#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/search/impl/search.hpp>

#include <teaser/ply_io.h>

#include <beam_matching/TeaserPPMatcher.h>
#include <beam_utils/math.h>
#include <beam_utils/pointclouds.h>

namespace beam_matching {

void LoadTeaserClouds(Eigen::Matrix<double, 3, Eigen::Dynamic>& cloud,
                      Eigen::Matrix<double, 3, Eigen::Dynamic>& cloud_pert,
                      const Eigen::Matrix4d& T_CLOUDPERT_CLOUD,
                      const std::string& scan_path) {
  // Load the .ply file
  teaser::PLYReader reader;
  teaser::PointCloud cloud_tmp;
  reader.read(scan_path, cloud_tmp);
  int N = cloud_tmp.size();

  // Convert the point cloud to Eigen
  cloud = Eigen::Matrix<double, 3, Eigen::Dynamic>(3, N);
  for (int i = 0; i < N; ++i) {
    cloud.col(i) << cloud_tmp[i].x, cloud_tmp[i].y, cloud_tmp[i].z;
  }

  // Homogeneous coordinates
  Eigen::Matrix<double, 4, Eigen::Dynamic> cloud_h;
  cloud_h.resize(4, cloud.cols());
  cloud_h.topRows(3) = cloud;
  cloud_h.bottomRows(1) = Eigen::Matrix<double, 1, Eigen::Dynamic>::Ones(N);

  // Apply transformation
  Eigen::Matrix<double, 4, Eigen::Dynamic> cloud_pert_h =
      T_CLOUDPERT_CLOUD * cloud_h;
  cloud_pert = cloud_pert_h.topRows(3);
}

PointCloudPtr EigenPointCloudToPCL(
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& cloud_eig) {
  PointCloudPtr cloud_pcl = std::make_shared<PointCloud>();
  for (int i = 0; i < cloud_eig.cols(); i++) {
    pcl::PointXYZ p;
    p.x = cloud_eig(0, i);
    p.y = cloud_eig(1, i);
    p.z = cloud_eig(2, i);
    cloud_pcl->push_back(p);
  }
  return cloud_pcl;
}

class Data {
public:
  Data() {
    // get file paths
    std::string test_path = __FILE__;
    std::string current_file = "teaserpp_tests.cpp";
    test_path.erase(test_path.end() - current_file.size(), test_path.end());
    // std::string scan_path1 = test_path + "data/testscan.pcd";
    std::string scan_path2 = test_path + "data/bun_zipper_res3.ply";
    std::string config_path = test_path + "config/teaserpp_config.json";

    // load matcher params
    params = TeaserPPMatcherParams(config_path);

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

    // NOTE: lidar scan too large for many computers
    // read lidar scan and transform
    // lidar_scan = std::make_shared<PointCloud>();
    // pcl::io::loadPCDFile(scan_path1, *lidar_scan);
    // lidar_scan_pert = std::make_shared<PointCloud>();
    // pcl::transformPointCloud(*lidar_scan, *lidar_scan_pert,
    //                          beam::InvertTransform(T_WORLD_CLOUD2));

    // read ply as teaser pointcloud
    LoadTeaserClouds(bunny, bunny_pert, beam::InvertTransform(T_WORLD_CLOUD2),
                     scan_path2);
  }

  Eigen::Matrix4d T_WORLD_CLOUD1;
  Eigen::Matrix4d T_WORLD_CLOUD2;
  TeaserPPMatcherParams params;
  // PointCloudPtr lidar_scan;
  // PointCloudPtr lidar_scan_pert;
  Eigen::Matrix<double, 3, Eigen::Dynamic> bunny;
  Eigen::Matrix<double, 3, Eigen::Dynamic> bunny_pert;
};

Data data_;

TEST_CASE("Test bunny registration with beam Matcher class") {
  PointCloudPtr bunny_pcl = EigenPointCloudToPCL(data_.bunny);
  PointCloudPtr bunny_pert_pcl = EigenPointCloudToPCL(data_.bunny_pert);

  // setup matcher
  TeaserPPMatcher matcher(data_.params);
  matcher.Setup(bunny_pcl, bunny_pert_pcl);

  // test and assert
  bool match_success = matcher.Match();
  REQUIRE(match_success == true);
  Eigen::Matrix4d T_CLOUD2_WORLD_measured = matcher.GetResult().matrix();
  const Eigen::Matrix4d& T = beam::InvertTransform(data_.T_WORLD_CLOUD2);
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      REQUIRE(std::abs(T(i, j) - T_CLOUD2_WORLD_measured(i, j)) < 0.001);
    }
  }
}

TEST_CASE("Test lidar scan registration with beam Matcher class") {
  // setup matcher
  PointCloudPtr bunny_pcl = EigenPointCloudToPCL(data_.bunny);
  PointCloudPtr bunny_pert_pcl = EigenPointCloudToPCL(data_.bunny_pert);

  TeaserPPMatcher matcher(data_.params);
  matcher.Setup(bunny_pcl, bunny_pert_pcl);

  // test and assert
  bool match_success = matcher.Match();
  REQUIRE(match_success == true);
  Eigen::Matrix4d T_CLOUD2_WORLD_measured = matcher.GetResult().matrix();
  const Eigen::Matrix4d& T = beam::InvertTransform(data_.T_WORLD_CLOUD2);
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      REQUIRE(std::abs(T(i, j) - T_CLOUD2_WORLD_measured(i, j)) < 0.001);
    }
  }
}

} // namespace beam_matching
