
#define CATCH_CONFIG_MAIN
#include <fstream>
#include <iostream>
#include <math.h>

#include <catch2/catch.hpp>

#include <beam_cv/geometry/PoseRefinement.h>

#include <beam_calibration/Radtan.h>

std::shared_ptr<beam_calibration::CameraModel> cam;

double randomInRange(double a, double b) {
  double r = (double)rand() / RAND_MAX;
  return a + r * (b - a);
}

void GenerateCorrespondences(std::shared_ptr<beam_calibration::CameraModel> cam,
                             std::vector<Eigen::Vector2i>& pixels,
                             std::vector<Eigen::Vector3d>& points, int n) {
  for (int i = 0; i < n; i++) {
    int x = rand() % cam->GetWidth();
    int y = rand() % cam->GetHeight();
    Eigen::Vector2i pixel(x, y);
    Eigen::Vector3d point = cam->BackProject(pixel).value();
    double depth_min = 1;
    double depth_max = 15;
    double scalar = randomInRange(depth_min, depth_max);
    point *= scalar;
    pixels.push_back(pixel);
    points.push_back(point);
  }
}

void PerturbCorrespondences(std::vector<Eigen::Vector2i>& pixels,
                            std::vector<Eigen::Vector3d>& points,
                            double pixel_pert, double point_pert) {
  for (size_t i = 0; i < pixels.size(); i++) {
    for (int j = 0; j < pixels[i].size(); j++) {
      double offset = randomInRange(-pixel_pert, pixel_pert);
    }
    for (int j = 0; j < points[i].size(); j++) {
      points[i][j] =
          randomInRange(points[i][j] - point_pert, points[i][j] + point_pert);
    }
  }
}

void PerturbPose(Eigen::Matrix4d& pose, double r_pert, double t_pert) {
  // perturb rotation
  Eigen::AngleAxisd roll(randomInRange(-r_pert, r_pert),
                         Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitch(randomInRange(-r_pert, r_pert),
                          Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw(randomInRange(-r_pert, r_pert),
                        Eigen::Vector3d::UnitX());
  Eigen::Quaterniond q = roll * pitch * yaw;
  pose.block(0, 0, 3, 3) = q.toRotationMatrix();
  // perturb translation
  pose(0, 3) = randomInRange(-t_pert, t_pert);
  pose(1, 3) = randomInRange(-t_pert, t_pert);
  pose(2, 3) = randomInRange(-t_pert, t_pert);
}

TEST_CASE("Refine given perfect correspondences, perfect pose.") {
  // make camera model (for all tests)
  std::string location = __FILE__;
  location.erase(location.end() - 31, location.end());
  std::string intrinsics_loc = location + "tests/test_data/K.json";
  cam = beam_calibration::CameraModel::Create(intrinsics_loc);

  // generate correspondences
  std::vector<Eigen::Vector2i> pixels;
  std::vector<Eigen::Vector3d> points;
  GenerateCorrespondences(cam, pixels, points, 30);

  // refine pose
  Eigen::Matrix4d estimate = Eigen::Matrix4d::Identity();
  beam_cv::PoseRefinement refiner;
  std::string report;
  Eigen::Matrix4d pose =
      refiner.RefinePose(estimate, cam, pixels, points, report);

  Eigen::Matrix4d truth = Eigen::Matrix4d::Identity();
  REQUIRE(pose.isApprox(truth, 1e-3));
}

TEST_CASE("Refine given perfect correspondences, perturbed pose.") {
  // generate correspondences
  std::vector<Eigen::Vector2i> pixels;
  std::vector<Eigen::Vector3d> points;
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
  Eigen::Matrix4d pose =
      refiner.RefinePose(estimate, cam, pixels, points, report);

  Eigen::Matrix4d truth = Eigen::Matrix4d::Identity();
  REQUIRE(pose.isApprox(truth, 1e-3));
}

TEST_CASE("Refine given perturbed pixels and points, perturbed pose.") {
  // generate correspondences
  std::vector<Eigen::Vector2i> pixels;
  std::vector<Eigen::Vector3d> points;
  GenerateCorrespondences(cam, pixels, points, 30);

  // randomly perturb pose to use as initial estimate
  Eigen::Matrix4d estimate = Eigen::Matrix4d::Identity();
  // set maximum perturbation (up to +/- x)
  double r_pert = M_PI / 10;
  double t_pert = .1;
  PerturbPose(estimate, r_pert, t_pert);

  // randomly perturb points/pixels (up to +/- x)
  double pixel_pert = 2;
  double point_pert = .05;
  PerturbCorrespondences(pixels, points, pixel_pert, point_pert);

  // refine pose
  beam_cv::PoseRefinement refiner;
  std::string report;
  Eigen::Matrix4d pose =
      refiner.RefinePose(estimate, cam, pixels, points, report);

  Eigen::Matrix4d truth = Eigen::Matrix4d::Identity();
  REQUIRE(pose.isApprox(truth, 1e-2));
}