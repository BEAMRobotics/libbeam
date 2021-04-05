#define CATCH_CONFIG_MAIN
#include <fstream>
#include <iostream>

#include <catch2/catch.hpp>

#include <beam_utils/time.h>

#include <beam_cv/Utils.h>
#include <beam_cv/geometry/AbsolutePoseEstimator.h>
#include <beam_cv/geometry/RelativePoseEstimator.h>
#include <beam_cv/geometry/Triangulation.h>

#include <beam_cv/descriptors/Descriptors.h>
#include <beam_cv/detectors/Detectors.h>
#include <beam_cv/matchers/Matchers.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <beam_calibration/Radtan.h>

void ReadMatches(std::string file, std::vector<Eigen::Vector2i>& matches1,
                 std::vector<Eigen::Vector2i>& matches2) {
  // declare variables
  std::ifstream infile;
  std::string line;
  // open file
  infile.open(file);
  // extract poses
  matches1.resize(0);
  matches2.resize(0);
  while (!infile.eof()) {
    // get timestamp k
    std::getline(infile, line, ',');
    int u1 = std::stod(line);
    std::getline(infile, line, ';');
    int v1 = std::stod(line);
    std::getline(infile, line, ',');
    int u2 = std::stod(line);
    std::getline(infile, line, '\n');
    int v2 = std::stod(line);
    Eigen::Vector2i p1{u1, v1};
    matches1.push_back(p1);
    Eigen::Vector2i p2{u2, v2};
    matches2.push_back(p2);
  }
}

void GenerateP3PMatches(std::shared_ptr<beam_calibration::CameraModel> cam,
                        std::vector<Eigen::Vector2i>& pixels,
                        std::vector<Eigen::Vector3d>& points, int n) {
  for (int i = 0; i < n; i++) {
    int x = rand() % cam->GetWidth();
    int y = rand() % cam->GetHeight();
    Eigen::Vector2i pixel(x, y);
    Eigen::Vector3d point = cam->BackProject(pixel).value();
    int scalar = rand() + 1;
    point *= scalar;
    pixels.push_back(pixel);
    points.push_back(point);
  }
}

TEST_CASE("Test triangulation -- fuse.") {
  std::string cam_loc = "/home/jake/data/VIO/euroc.json";
  std::string im_R_loc = "/home/jake/im1.jpg";
  std::string im_L_loc = "/home/jake/im2.jpg";

  Eigen::Matrix4d P_vicon1_world;
  P_vicon1_world << 0.969841, 0.242046, 0.0286742, 1.19256, //
      -0.242298, 0.970185, 0.00562983, 2.41577,             //
      -0.0264566, -0.0124077, 0.999573, 1.41054,            //
      0, 0, 0, 1;                                           //

  Eigen::Matrix4d P_vicon2_world;
  P_vicon2_world << 0.870246, 0.491662, 0.0306658, 1.30745, //
      -0.491824, 0.870691, -0.00253623, 2.47103,            //
      -0.0279474, -0.012875, 0.999526, 1.38097,             //
      0, 0, 0, 1;                                           //

  Eigen::Matrix4d T_body_vicon;
  T_body_vicon << 0.33638, -0.01749, 0.94156, 0.06901, //
      -0.02078, -0.99972, -0.01114, -0.02781,          //
      0.94150, -0.01582, -0.33665, -0.12395,           //
      0.0, 0.0, 0.0, 1.0;                              //

  Eigen::Matrix4d P_body_world_1 = T_body_vicon * P_vicon1_world.inverse();
  Eigen::Matrix4d P_body_world_2 = T_body_vicon * P_vicon2_world.inverse();

  Eigen::Matrix4d T_body_cam0;
  T_body_cam0 << 0.014865542, -0.99988092, 0.0041402967, -0.021640145, //
      0.99955724, 0.014967213, 0.02571552, -0.06467698,                //
      -0.025774436, 0.0037561883, 0.99966072, 0.0098107305,            //
      0.0, 0.0, 0.0, 1.0;                                              //

  Eigen::Matrix4d P_cam0_world_1 = T_body_cam0.inverse() * P_body_world_1;
  Eigen::Matrix4d P_cam0_world_2 = T_body_cam0.inverse() * P_body_world_2;

  std::cout << P_cam0_world_1 << std::endl;
  std::cout << P_cam0_world_2 << std::endl;

  Eigen::Matrix4d relative = P_cam0_world_1 * P_cam0_world_2.inverse();
  std::cout << relative << std::endl;

  // Eigen::Matrix4d T_cam0_vicon = T_body_cam0.inverse() * T_body_vicon;

  // Eigen::Matrix4d Pl = T_cam0_vicon * P_vicon1_R;
  // Eigen::Matrix4d Pr = T_cam0_vicon * P_vicon2_R;
  // std::cout << Pl << std::endl;
  // std::cout << Pr << std::endl;
  // Eigen::Matrix4d relative = Pr * Pl.inverse();
  // std::cout << relative << std::endl;

  std::shared_ptr<beam_cv::Matcher> matcher =
      std::make_shared<beam_cv::FLANNMatcher>(beam_cv::FLANN::KDTree, 0.8, true,
                                              true, cv::FM_RANSAC, 5);
  std::shared_ptr<beam_cv::Descriptor> descriptor =
      std::make_shared<beam_cv::ORBDescriptor>();
  std::shared_ptr<beam_cv::Detector> detector =
      std::make_shared<beam_cv::FASTDetector>();

  std::shared_ptr<beam_calibration::CameraModel> cam =
      beam_calibration::CameraModel::Create(cam_loc);

  std::vector<Eigen::Vector2i> pL_v;
  std::vector<Eigen::Vector2i> pR_v;
  cv::Mat imL = cv::imread(im_L_loc, cv::IMREAD_GRAYSCALE);
  cv::Mat imR = cv::imread(im_R_loc, cv::IMREAD_GRAYSCALE);
  beam_cv::DetectComputeAndMatch(imL, imR, descriptor, detector, matcher, pL_v,
                                 pR_v);

  Eigen::Matrix4d Pr = P_cam0_world_1;
  Eigen::Matrix4d Pl = P_cam0_world_2;

  // Pr << 0.963895, -0.0892999, 0.25086, -0.06016, //
  //     0.0881189, 0.995982, 0.0159599, -0.01026,  //
  //     -0.25128, 0.00671998, 0.967892, 0.115776,  //
  //     0, 0, 0, 1;                                //

  // Pr << 0.963895, -0.0892999, 0.25086, -0.414332, //
  //     0.0881189, 0.995982, 0.0159599, 0.18754,    //
  //     -0.25128, 0.00671998, 0.967892, -0.462482,  //
  //     0, 0, 0, 1;                                 //

  //   goal relative pose:
  //   0.967982  -0.106405   0.227351  -0.074342
  //   0.105708    0.99428  0.0152759   0.100705
  //  -0.227676 0.00924609   0.973693   0.992135
  //          0          0          0          1

  std::vector<beam::opt<Eigen::Vector3d>> points =
      beam_cv::Triangulation::TriangulatePoints(cam, cam, Pl, Pr, pL_v, pR_v);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (beam::opt<Eigen::Vector3d> p : points) {
    if (p.has_value()) {
      pcl::PointXYZ pclpoint;
      pclpoint.x = p.value()[0];
      pclpoint.y = p.value()[1];
      pclpoint.z = p.value()[2];
      cloud->points.push_back(pclpoint);
    }
  }
  pcl::io::savePCDFileBinary("/home/jake/visual_map.pcd", *cloud);
  int num_inliers = beam_cv::CheckInliers(cam, cam, pL_v, pR_v, Pl, Pr, 5);
  INFO(num_inliers);
  REQUIRE(num_inliers == 100);
}

// TEST_CASE("Test triangulation.") {
//   std::string cam_loc = __FILE__;
//   cam_loc.erase(cam_loc.end() - 24, cam_loc.end());
//   cam_loc += "tests/test_data/K.json";
//   std::shared_ptr<beam_calibration::CameraModel> cam =
//       beam_calibration::CameraModel::Create(cam_loc);

//   Eigen::Matrix4d Pr = Eigen::Matrix4d::Identity();
//   Eigen::Matrix4d Pc;
//   Pc << 0.994638, 0.0300318, 0.0989638, -0.915986, //
//       -0.0315981, 0.999398, 0.0142977, -0.134433,  //
//       -0.0984749, -0.0173481, 0.994988, -0.378019, //
//       0, 0, 0, 1;                                  //

//   std::string matches_loc = __FILE__;
//   matches_loc.erase(matches_loc.end() - 24, matches_loc.end());
//   matches_loc += "tests/test_data/matches.txt";
//   std::vector<Eigen::Vector2i> frame1_matches;
//   std::vector<Eigen::Vector2i> frame2_matches;
//   ReadMatches(matches_loc, frame1_matches, frame2_matches);

//   int num_inliers = beam_cv::CheckInliers(cam, cam, frame1_matches,
//                                           frame2_matches, Pr, Pc, 5);
//   INFO(num_inliers);
//   REQUIRE(num_inliers == 100);
// }

// TEST_CASE("Test 8 point Relative Pose Estimator.") {
//   Eigen::Matrix4d P;
//   P << 0.994502, -0.0321403, -0.0996619, 0.872111, 0.0304609, 0.999368,
//       -0.0183281, 0.154409, 0.100188, 0.0151916, 0.994853, 0.464306, 0, 0, 0,
//       1;

//   std::string cam_loc = __FILE__;
//   cam_loc.erase(cam_loc.end() - 24, cam_loc.end());
//   cam_loc += "tests/test_data/K.json";
//   std::shared_ptr<beam_calibration::CameraModel> cam =
//       beam_calibration::CameraModel::Create(cam_loc);

//   std::string matches_loc = __FILE__;
//   matches_loc.erase(matches_loc.end() - 24, matches_loc.end());
//   matches_loc += "tests/test_data/matches.txt";
//   // extract poses
//   std::vector<Eigen::Vector2i> frame1_matches;
//   std::vector<Eigen::Vector2i> frame2_matches;
//   ReadMatches(matches_loc, frame1_matches, frame2_matches);
//   beam::opt<Eigen::Matrix3d> E =
//       beam_cv::RelativePoseEstimator::EssentialMatrix8Point(
//           cam, cam, frame1_matches, frame2_matches);
//   std::vector<Eigen::Matrix3d> R;
//   std::vector<Eigen::Vector3d> t;
//   beam_cv::RelativePoseEstimator::RtFromE(E.value(), R, t);
//   beam::opt<Eigen::Matrix4d> pose;
//   beam_cv::RelativePoseEstimator::RecoverPose(cam, cam, frame1_matches,
//                                               frame2_matches, R, t,
//                                               pose, 10.0);

//   REQUIRE(pose.value().isApprox(P, 1e-4));
// }

// TEST_CASE("Test RANSAC Relative Pose estimator - 7 Point") {
//   struct timespec t;

//   std::string cam_loc = __FILE__;
//   cam_loc.erase(cam_loc.end() - 24, cam_loc.end());
//   cam_loc += "tests/test_data/K.json";
//   std::shared_ptr<beam_calibration::CameraModel> cam =
//       beam_calibration::CameraModel::Create(cam_loc);
//   std::string matches_loc = __FILE__;
//   matches_loc.erase(matches_loc.end() - 24, matches_loc.end());
//   matches_loc += "tests/test_data/matches.txt";

//   // extract poses
//   std::vector<Eigen::Vector2i> frame1_matches;
//   std::vector<Eigen::Vector2i> frame2_matches;
//   ReadMatches(matches_loc, frame1_matches, frame2_matches);
//   BEAM_INFO("Starting 7 Point RANSAC");
//   beam::tic(&t);
//   beam::opt<Eigen::Matrix4d> pose =
//       beam_cv::RelativePoseEstimator::RANSACEstimator(
//           cam, cam, frame1_matches, frame2_matches,
//           beam_cv::EstimatorMethod::SEVENPOINT, 20, 5, 13);
//   float elapsed = beam::toc(&t);
//   BEAM_INFO("7 Point RANSAC elapsed time (20 iterations): {}", elapsed);
//   Eigen::Matrix4d Pr = Eigen::Matrix4d::Identity();
//   int num_inliers = beam_cv::CheckInliers(cam, cam, frame1_matches,
//                                           frame2_matches, Pr, pose.value(),
//                                           5);
//   INFO(num_inliers);
//   REQUIRE(num_inliers == 100);
// }

// TEST_CASE("Test RANSAC Relative Pose estimator - 8 Point") {
//   struct timespec t;

//   std::string cam_loc = __FILE__;
//   cam_loc.erase(cam_loc.end() - 24, cam_loc.end());
//   cam_loc += "tests/test_data/K.json";
//   std::shared_ptr<beam_calibration::CameraModel> cam =
//       beam_calibration::CameraModel::Create(cam_loc);
//   std::string matches_loc = __FILE__;
//   matches_loc.erase(matches_loc.end() - 24, matches_loc.end());
//   matches_loc += "tests/test_data/matches.txt";

//   // extract poses
//   std::vector<Eigen::Vector2i> frame1_matches;
//   std::vector<Eigen::Vector2i> frame2_matches;
//   ReadMatches(matches_loc, frame1_matches, frame2_matches);
//   BEAM_INFO("Starting 8 Point RANSAC");
//   beam::tic(&t);
//   beam::opt<Eigen::Matrix4d> pose =
//       beam_cv::RelativePoseEstimator::RANSACEstimator(
//           cam, cam, frame1_matches, frame2_matches,
//           beam_cv::EstimatorMethod::EIGHTPOINT, 200, 5, 123);
//   float elapsed = beam::toc(&t);
//   BEAM_INFO("8 Point RANSAC elapsed time (200 iterations): {}", elapsed);
//   Eigen::Matrix4d Pr = Eigen::Matrix4d::Identity();
//   int num_inliers = beam_cv::CheckInliers(cam, cam, frame1_matches,
//                                           frame2_matches, Pr, pose.value(),
//                                           10);
//   INFO(num_inliers);
//   REQUIRE(num_inliers == 49);
// }

// TEST_CASE("Test P3P Absolute Pose Estimator") {
//   Eigen::Matrix4d truth = Eigen::Matrix4d::Identity();

//   // make camera model
//   std::string location = __FILE__;
//   location.erase(location.end() - 24, location.end());
//   std::string intrinsics_loc = location + "tests/test_data/K.json";
//   std::shared_ptr<beam_calibration::CameraModel> cam =
//       beam_calibration::CameraModel::Create(intrinsics_loc);

//   // generate 3 correspondences
//   std::vector<Eigen::Vector2i> pixels;
//   std::vector<Eigen::Vector3d> points;
//   GenerateP3PMatches(cam, pixels, points, 3);

//   // find the pose
//   std::vector<Eigen::Matrix4d> poses =
//       beam_cv::AbsolutePoseEstimator::P3PEstimator(cam, pixels, points);

//   // check if the solution was found
//   bool check = false;
//   for (size_t i = 0; i < poses.size(); i++) {
//     if (poses[i].isApprox(truth, 1e-3)) { check = true; }
//   }

//   REQUIRE(check);
// }

// TEST_CASE("Test RANSAC Absolute Pose estimator.") {
//   Eigen::Matrix4d truth = Eigen::Matrix4d::Identity();

//   // make camera model
//   std::string location = __FILE__;
//   location.erase(location.end() - 24, location.end());
//   std::string intrinsics_loc = location + "tests/test_data/K.json";
//   std::shared_ptr<beam_calibration::CameraModel> cam =
//       beam_calibration::CameraModel::Create(intrinsics_loc);

//   // generate correspondences
//   std::vector<Eigen::Vector2i> pixels;
//   std::vector<Eigen::Vector3d> points;
//   GenerateP3PMatches(cam, pixels, points, 30);

//   // add some noise
//   for (size_t i = 0; i < pixels.size(); i++) {
//     if (i % 2 == 0) {
//       points[i](0) += 1000;
//       points[i](1) -= 1000;
//       points[i](2) *= 1000;
//     }
//   }

//   struct timespec t;

//   int seed = rand();
//   beam::tic(&t);
//   Eigen::Matrix4d pose = beam_cv::AbsolutePoseEstimator::RANSACEstimator(
//       cam, pixels, points, 30, 5, seed);
//   float elapsed = beam::toc(&t);
//   BEAM_INFO("RANSAC PnP (30 iterations): {}", elapsed);

//   REQUIRE(pose.isApprox(truth, 1e-3));
// }