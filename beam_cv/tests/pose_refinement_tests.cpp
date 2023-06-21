#define CATCH_CONFIG_MAIN

#include <beam_cv/geometry/PoseRefinement.h>

#include <fstream>
#include <iostream>
#include <math.h>

#include <catch2/catch.hpp>

#include <beam_calibration/Radtan.h>
#include <beam_utils/math.h>
#include <beam_utils/se3.h>
#include <beam_utils/time.h>

std::shared_ptr<beam_calibration::CameraModel> cam;

void GenerateCorrespondences(
    std::shared_ptr<beam_calibration::CameraModel> cam,
    std::vector<Eigen::Vector2i, beam::AlignVec2i>& pixels,
    std::vector<Eigen::Vector3d, beam::AlignVec3d>& points, int n) {
  for (int i = 0; i < n; i++) {
    int x = rand() % cam->GetWidth();
    int y = rand() % cam->GetHeight();
    Eigen::Vector2i pixel(x, y);
    Eigen::Vector3d point;
    cam->BackProject(pixel, point);
    double depth_min = 1;
    double depth_max = 15;
    double scalar = beam::randf(depth_max, depth_min);
    point *= scalar;
    pixels.push_back(pixel);
    points.push_back(point);
  }
}

void PerturbCorrespondences(
    std::vector<Eigen::Vector2i, beam::AlignVec2i>& pixels,
    std::vector<Eigen::Vector3d, beam::AlignVec3d>& points, int pixel_pert,
    double point_pert) {
  for (size_t i = 0; i < pixels.size(); i++) {
    for (int j = 0; j < pixels[i].size(); j++) {
      pixels[i][j] =
          beam::randi(pixels[i][j] + pixel_pert, pixels[i][j] - pixel_pert);
    }
    for (int j = 0; j < points[i].size(); j++) {
      points[i][j] =
          beam::randf(points[i][j] + point_pert, points[i][j] - point_pert);
    }
  }
}

void ReadCorrespondences(
    std::vector<Eigen::Vector2i, beam::AlignVec2i>& pixels1,
    std::vector<Eigen::Vector3d, beam::AlignVec3d>& points1,
    Eigen::Matrix4d& pose1,
    std::vector<Eigen::Vector2i, beam::AlignVec2i>& pixels2,
    std::vector<Eigen::Vector3d, beam::AlignVec3d>& points2,
    Eigen::Matrix4d& pose2) {
  std::string location = __FILE__;
  location.erase(location.end() - 31, location.end());
  std::string corr_file1 = location + "tests/test_data/im1_correspondences.txt";
  std::string corr_file2 = location + "tests/test_data/im2_correspondences.txt";
  std::cout << corr_file2 << std::endl;

  pose1 = Eigen::Matrix4d::Zero();
  pose1 << 0.998964, -0.0365393, 0.027119, -0.0609512, -0.0400579, -0.423459,
      0.905029, -0.00768064, -0.0215854, -0.905178, -0.424484, -0.186287, 0, 0,
      0, 1;
  pose2 = Eigen::Matrix4d::Zero();
  pose2 << 0.999055, -0.0359871, 0.024385, -0.0601773, -0.0375938, -0.433601,
      0.90032, -0.00260963, -0.0218265, -0.900386, -0.434544, -0.195233, 0, 0,
      0, 1;

  std::ifstream infile;
  std::string line;
  // open file
  infile.open(corr_file1);
  // extract poses
  pixels1.resize(0);
  points1.resize(0);
  while (!infile.eof()) {
    // get timestamp k
    std::getline(infile, line, ';');
    int u = std::stod(line);
    std::getline(infile, line, '|');
    int v = std::stod(line);
    std::getline(infile, line, ';');
    double x = std::stod(line);
    std::getline(infile, line, ';');
    double y = std::stod(line);
    std::getline(infile, line, '\n');
    double z = std::stod(line);
    Eigen::Vector2i pixel{u, v};
    pixels1.push_back(pixel);
    Eigen::Vector3d point{x, y, z};
    points1.push_back(point);
  }

  std::ifstream infile2;
  std::string line2;
  infile2.open(corr_file2);
  // extract poses
  pixels2.resize(0);
  points2.resize(0);
  while (!infile2.eof()) {
    // get timestamp k
    std::getline(infile2, line2, ';');
    int u = std::stod(line2);
    std::getline(infile2, line2, '|');
    int v = std::stod(line2);
    std::getline(infile2, line2, ';');
    double x = std::stod(line2);
    std::getline(infile2, line2, ';');
    double y = std::stod(line2);
    std::getline(infile2, line2, '\n');
    double z = std::stod(line2);
    Eigen::Vector2i pixel{u, v};
    pixels2.push_back(pixel);
    Eigen::Vector3d point{x, y, z};
    points2.push_back(point);
  }
}

void PerturbPose(Eigen::Matrix4d& pose, double r_pert, double t_pert) {
  // generate random perturbations
  Eigen::Matrix<double, 6, 1> perturbations;
  perturbations(0) = beam::randf(r_pert, -r_pert);
  perturbations(1) = beam::randf(r_pert, -r_pert);
  perturbations(2) = beam::randf(r_pert, -r_pert);
  perturbations(3) = beam::randf(t_pert, -t_pert);
  perturbations(4) = beam::randf(t_pert, -t_pert);
  perturbations(5) = beam::randf(t_pert, -t_pert);

  pose = beam::PerturbTransformRadM(pose, perturbations);
}

TEST_CASE("Euroc VIO Refinement Tests.") {
  // make camera model (for just this test)
  std::string location = __FILE__;
  location.erase(location.end() - 31, location.end());
  std::string intrinsics_loc = location + "tests/test_data/euroc.json";
  std::shared_ptr<beam_calibration::CameraModel> camera =
      beam_calibration::CameraModel::Create(intrinsics_loc);

  // generate correspondences
  std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels1, pixels2;
  std::vector<Eigen::Vector3d, beam::AlignVec3d> points1, points2;
  Eigen::Matrix4d pose1, pose2;
  ReadCorrespondences(pixels1, points1, pose1, pixels2, points2, pose2);

  beam_cv::PoseRefinement refiner;
  std::string report1, report2;
  // refine pose 1
  Eigen::Matrix4d refined_pose1 = refiner.RefinePose(
      pose1, camera, pixels1, points1, nullptr, nullptr, report1);
  Eigen::Matrix4d target_refined1;
  target_refined1 << 0.999415, -0.0292887, 0.0176378, -0.0568712, -0.0262657,
      -0.32748, 0.944493, -0.00156549, -0.021887, -0.944404, -0.328058,
      -0.0443409, 0, 0, 0, 1;

  REQUIRE(refined_pose1.isApprox(target_refined1, 1e-3));
  // refine pose 2
  Eigen::Matrix4d refined_pose2 = refiner.RefinePose(
      pose2, camera, pixels2, points2, nullptr, nullptr, report2);
  Eigen::Matrix4d target_refined2;
  target_refined2 << 0.999323, -0.0335793, 0.0150601, -0.0582982, -0.0249295,
      -0.316631, 0.94822, -0.00263348, -0.027072, -0.947953, -0.317254,
      -0.0448607, 0, 0, 0, 1;
  REQUIRE(refined_pose2.isApprox(target_refined2, 1e-3));

  // refine pose 2 without removing bad points
  refined_pose2 = refiner.RefinePose(pose2, camera, pixels2, points2, nullptr,
                                     nullptr, report2, false);
  REQUIRE(refined_pose2.isApprox(pose2, 1e-4));
}

TEST_CASE("Refine given perfect correspondences, perfect pose.") {
  // make camera model (for all tests)
  std::string location = __FILE__;
  location.erase(location.end() - 31, location.end());
  std::string intrinsics_loc = location + "tests/test_data/K.json";
  cam = beam_calibration::CameraModel::Create(intrinsics_loc);

  // generate correspondences
  std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
  std::vector<Eigen::Vector3d, beam::AlignVec3d> points;
  GenerateCorrespondences(cam, pixels, points, 30);

  // refine pose
  Eigen::Matrix4d estimate = Eigen::Matrix4d::Identity();
  beam_cv::PoseRefinement refiner;
  std::string report;
  Eigen::Matrix4d pose = refiner.RefinePose(estimate, cam, pixels, points,
                                            nullptr, nullptr, report);

  Eigen::Matrix4d truth = Eigen::Matrix4d::Identity();
  REQUIRE(pose.isApprox(truth, 1e-3));
}

TEST_CASE("Refine given perfect correspondences, perturbed pose.") {
  // generate correspondences
  std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
  std::vector<Eigen::Vector3d, beam::AlignVec3d> points;
  GenerateCorrespondences(cam, pixels, points, 30);

  // randomly perturb pose to use as initial estimate
  Eigen::Matrix4d estimate = Eigen::Matrix4d::Identity();
  // set maximum perturbation (up to +/- x)
  double r_pert = M_PI / 10;
  double t_pert = .1;
  PerturbPose(estimate, r_pert, t_pert);

  // refine pose
  beam_cv::PoseRefinement refiner;
  std::string report;
  Eigen::Matrix4d pose = refiner.RefinePose(estimate, cam, pixels, points,
                                            nullptr, nullptr, report);

  Eigen::Matrix4d truth = Eigen::Matrix4d::Identity();
  REQUIRE(pose.isApprox(truth, 1e-3));
}

TEST_CASE("Refine given perturbed pixels and points, perturbed pose.") {
  // generate correspondences
  std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
  std::vector<Eigen::Vector3d, beam::AlignVec3d> points;
  GenerateCorrespondences(cam, pixels, points, 30);

  // randomly perturb pose to use as initial estimate
  Eigen::Matrix4d estimate = Eigen::Matrix4d::Identity();
  // set maximum perturbation (up to +/- x)
  double r_pert = M_PI / 10;
  double t_pert = .1;
  PerturbPose(estimate, r_pert, t_pert);

  // randomly perturb points/pixels (up to +/- x)
  int pixel_pert = 2;
  double point_pert = .05;
  PerturbCorrespondences(pixels, points, pixel_pert, point_pert);

  // refine pose
  beam_cv::PoseRefinement refiner;
  std::string report;
  Eigen::Matrix4d pose = refiner.RefinePose(estimate, cam, pixels, points,
                                            nullptr, nullptr, report);

  Eigen::Matrix4d truth = Eigen::Matrix4d::Identity();
  REQUIRE(pose.isApprox(truth, 1e-2));
}

TEST_CASE(
    "Refine given perturbed pixels and points, perturbed pose, and a prior.") {
  // generate correspondences
  std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
  std::vector<Eigen::Vector3d, beam::AlignVec3d> points;
  GenerateCorrespondences(cam, pixels, points, 30);

  // randomly perturb pose to use as initial estimate
  Eigen::Matrix4d estimate = Eigen::Matrix4d::Identity();
  // set maximum perturbation (up to +/- x)
  double r_pert = M_PI / 10;
  double t_pert = .1;
  PerturbPose(estimate, r_pert, t_pert);

  // randomly perturb points/pixels (up to +/- x)
  int pixel_pert = 2;
  double point_pert = .05;
  PerturbCorrespondences(pixels, points, pixel_pert, point_pert);

  // refine pose
  beam_cv::PoseRefinement refiner;
  std::string report;
  std::shared_ptr<Eigen::Matrix<double, 6, 6>> A_out =
      std::make_shared<Eigen::Matrix<double, 6, 6>>();
  std::shared_ptr<Eigen::Matrix<double, 6, 6>> A =
      std::make_shared<Eigen::Matrix<double, 6, 6>>();
  *A = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix4d pose =
      refiner.RefinePose(estimate, cam, pixels, points, A, A_out, report);

  Eigen::Matrix4d truth = Eigen::Matrix4d::Identity();
  REQUIRE(pose.isApprox(truth, 1e-1));
}