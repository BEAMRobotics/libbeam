#define CATCH_CONFIG_MAIN
#include <fstream>
#include <iostream>

#include <catch2/catch.hpp>

#include <beam_cv/geometry/RelativePoseEstimator.h>
#include <beam_cv/geometry/Triangulation.h>

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
  int num_inliers = beam_cv::RelativePoseEstimator::CheckInliers(
      cam, cam, frame1_matches, frame2_matches, Pc, 5);
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
  Eigen::Matrix4d pose = beam_cv::RelativePoseEstimator::RecoverPose(
      cam, cam, frame1_matches, frame2_matches, R, t);

  REQUIRE(pose.isApprox(P, 1e-4));
}


TEST_CASE("RANSAC Pose estimator.") {
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
      beam_cv::EstimatorMethod::EIGHTPOINT, 100, 5, 123);
  
  int num_inliers = beam_cv::RelativePoseEstimator::CheckInliers(
      cam, cam, frame1_matches, frame2_matches, pose.value(), 5);
  INFO(num_inliers);
  REQUIRE(num_inliers == 22);

}
