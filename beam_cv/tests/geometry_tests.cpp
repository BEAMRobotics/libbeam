#define CATCH_CONFIG_MAIN
#include <fstream>
#include <iostream>

#include <catch2/catch.hpp>

#include <beam_cv/Utils.h>
#include <beam_cv/geometry/AbsolutePoseEstimator.h>
#include <beam_cv/geometry/RelativePoseEstimator.h>
#include <beam_cv/geometry/Triangulation.h>

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

TEST_CASE("Test triangulation.") {
  std::string cam_loc = __FILE__;
  cam_loc.erase(cam_loc.end() - 24, cam_loc.end());
  cam_loc += "tests/test_data/K.json";
  std::shared_ptr<beam_calibration::CameraModel> cam =
      beam_calibration::CameraModel::Create(cam_loc);

  Eigen::Matrix4d Pr = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d Pc;
  Pc << 0.996398, -0.022907, -0.0816518, 0.733184, 0.0231435, 0.99973,
      0.00195076, 0.0864691, 0.0815851, -0.00383344, 0.996659, 0.67451, 0, 0, 0,
      1;

  std::string matches_loc = __FILE__;
  matches_loc.erase(matches_loc.end() - 24, matches_loc.end());
  matches_loc += "tests/test_data/matches.txt";
  std::vector<Eigen::Vector2i> frame1_matches;
  std::vector<Eigen::Vector2i> frame2_matches;
  ReadMatches(matches_loc, frame1_matches, frame2_matches);
  int num_inliers = beam_cv::CheckInliers(cam, cam, frame1_matches,
                                          frame2_matches, Pr, Pc, 5);
  INFO(num_inliers);
  REQUIRE(num_inliers == 25);
}

TEST_CASE("Test 8 point Relative Pose Estimator.") {
  Eigen::Matrix4d P;
  P << 0.994502, -0.0321403, -0.0996619, 0.872111, 0.0304609, 0.999368,
      -0.0183281, 0.154409, 0.100188, 0.0151916, 0.994853, 0.464306, 0, 0, 0, 1;

  std::string cam_loc = __FILE__;
  cam_loc.erase(cam_loc.end() - 24, cam_loc.end());
  cam_loc += "tests/test_data/K.json";
  std::shared_ptr<beam_calibration::CameraModel> cam =
      beam_calibration::CameraModel::Create(cam_loc);

  std::string matches_loc = __FILE__;
  matches_loc.erase(matches_loc.end() - 24, matches_loc.end());
  matches_loc += "tests/test_data/matches.txt";
  // extract poses
  std::vector<Eigen::Vector2i> frame1_matches;
  std::vector<Eigen::Vector2i> frame2_matches;
  ReadMatches(matches_loc, frame1_matches, frame2_matches);
  opt<Eigen::Matrix3d> E =
      beam_cv::RelativePoseEstimator::EssentialMatrix8Point(
          cam, cam, frame1_matches, frame2_matches);
  std::vector<Eigen::Matrix3d> R;
  std::vector<Eigen::Vector3d> t;
  beam_cv::RelativePoseEstimator::RtFromE(E.value(), R, t);
  opt<Eigen::Matrix4d> pose = beam_cv::RelativePoseEstimator::RecoverPose(
      cam, cam, frame1_matches, frame2_matches, R, t);

  REQUIRE(pose.value().isApprox(P, 1e-4));
}

TEST_CASE("Test RANSAC Relative Pose estimator.") {
  std::string cam_loc = __FILE__;
  cam_loc.erase(cam_loc.end() - 24, cam_loc.end());
  cam_loc += "tests/test_data/K.json";
  std::shared_ptr<beam_calibration::CameraModel> cam =
      beam_calibration::CameraModel::Create(cam_loc);
  std::string matches_loc = __FILE__;
  matches_loc.erase(matches_loc.end() - 24, matches_loc.end());
  matches_loc += "tests/test_data/matches.txt";

  // extract poses
  std::vector<Eigen::Vector2i> frame1_matches;
  std::vector<Eigen::Vector2i> frame2_matches;
  ReadMatches(matches_loc, frame1_matches, frame2_matches);
  opt<Eigen::Matrix4d> pose = beam_cv::RelativePoseEstimator::RANSACEstimator(
      cam, cam, frame1_matches, frame2_matches,
      beam_cv::EstimatorMethod::EIGHTPOINT, 200, 5, 123);
  Eigen::Matrix4d Pr = Eigen::Matrix4d::Identity();
  int num_inliers = beam_cv::CheckInliers(cam, cam, frame1_matches,
                                          frame2_matches, Pr, pose.value(), 10);
  INFO(num_inliers);
  REQUIRE(num_inliers == 41);
}

TEST_CASE("Test P3P Absolute Pose Estimator") {
  Eigen::Matrix4d truth = Eigen::Matrix4d::Identity();

  // make camera model
  std::string location = __FILE__;
  location.erase(location.end() - 24, location.end());
  std::string intrinsics_loc = location + "tests/test_data/K.json";
  std::shared_ptr<beam_calibration::CameraModel> cam =
      beam_calibration::CameraModel::Create(intrinsics_loc);

  // generate 3 correspondences
  std::vector<Eigen::Vector2i> pixels;
  std::vector<Eigen::Vector3d> points;
  GenerateP3PMatches(cam, pixels, points, 3);

  // find the pose
  std::vector<Eigen::Matrix4d> poses =
      beam_cv::AbsolutePoseEstimator::P3PEstimator(cam, pixels, points);

  // check if the solution was found
  bool check = false;
  for (size_t i = 0; i < poses.size(); i++) {
    if (poses[i].isApprox(truth, 1e-3)) { check = true; }
  }

  REQUIRE(check);
}

TEST_CASE("Test RANSAC Absolute Pose estimator.") {
  Eigen::Matrix4d truth = Eigen::Matrix4d::Identity();

  // make camera model
  std::string location = __FILE__;
  location.erase(location.end() - 24, location.end());
  std::string intrinsics_loc = location + "tests/test_data/K.json";
  std::shared_ptr<beam_calibration::CameraModel> cam =
      beam_calibration::CameraModel::Create(intrinsics_loc);

  // generate correspondences
  std::vector<Eigen::Vector2i> pixels;
  std::vector<Eigen::Vector3d> points;
  GenerateP3PMatches(cam, pixels, points, 30);

  // add some noise
  for (size_t i = 0; i < pixels.size(); i++) {
    if (i % 2 == 0) {
      points[i](0) += 1000;
      points[i](1) -= 1000;
      points[i](2) *= 1000;
    }
  }

  int seed = rand();
  Eigen::Matrix4d pose = beam_cv::AbsolutePoseEstimator::RANSACEstimator(
      cam, pixels, points, 30, 5, seed);

  REQUIRE(pose.isApprox(truth, 1e-3));
}