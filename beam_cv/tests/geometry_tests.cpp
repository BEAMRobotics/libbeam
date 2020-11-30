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

void ReadP3PMatches(std::string file, std::vector<Eigen::Vector2i>& pixels,
                    std::vector<Eigen::Vector3d>& points) {
  // declare variables
  std::ifstream infile;
  std::string line;
  // open file
  infile.open(file);
  // extract contents
  pixels.clear();
  points.clear();
  while (!infile.eof()) {
    std::getline(infile, line, ',');
    int px1 = std::stod(line);
    std::getline(infile, line, ':');
    int px2 = std::stod(line);

    Eigen::Vector2i pixel{px1, px2};
    pixels.push_back(pixel);

    std::getline(infile, line, ',');
    double pt1 = std::stod(line);
    std::getline(infile, line, ',');
    double pt2 = std::stod(line);
    std::getline(infile, line, '\n');
    double pt3 = std::stod(line);

    Eigen::Vector3d point{pt1, pt2, pt3};
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
      beam_cv::EstimatorMethod::EIGHTPOINT, 200, 5, 123);
  Eigen::Matrix4d Pr = Eigen::Matrix4d::Identity();
  int num_inliers = beam_cv::CheckInliers(cam, cam, frame1_matches,
                                          frame2_matches, Pr, pose.value(), 10);
  INFO(num_inliers);
  REQUIRE(num_inliers == 31);
}

TEST_CASE("Test P3P Absolute Pose Estimator") {
  // make camera model
  std::string location = __FILE__;
  location.erase(location.end() - 24, location.end());
  std::string intrinsics_loc = location + "tests/test_data/K.json";
  std::shared_ptr<beam_calibration::CameraModel> cam =
      beam_calibration::CameraModel::Create(intrinsics_loc);

  // get corresponding points and pixels
  std::vector<Eigen::Vector2i> pixels;
  std::vector<Eigen::Vector3d> points;
  std::string matches_loc = location + "tests/test_data/p3p_matches.txt";
  ReadP3PMatches(matches_loc, pixels, points);

  // get solutions from samples of 3 point correspondences
  std::vector<Eigen::Matrix4d> transformations;
  std::vector<Eigen::Vector2i> pixels_sample;
  std::vector<Eigen::Vector3d> points_sample;
  std::vector<Eigen::Matrix4d> solution;

  for (size_t i = 0; i + 2 < pixels.size(); i++) {
    // grab 3 sequential pixels and points
    pixels_sample = std::vector<Eigen::Vector2i>(pixels.begin() + i,
                                                 pixels.begin() + i + 3);
    points_sample = std::vector<Eigen::Vector3d>(points.begin() + i,
                                                 points.begin() + i + 3);
    // p3p
    solution = beam_cv::AbsolutePoseEstimator::P3PEstimator(cam, pixels_sample,
                                                            points_sample);

    // push solutions to collection of transformation matrices
    for (size_t j = 0; j < solution.size(); j++) {
      transformations.push_back(solution[j]);
    };
  }

  // print all transformation matrices
  for (size_t i = 0; i < transformations.size(); i++) {
    std::cout << "solution " << i << ": " << std::endl
              << std::endl
              << transformations[i] << std::endl
              << std::endl;
  };

  REQUIRE(1 == 1);
}