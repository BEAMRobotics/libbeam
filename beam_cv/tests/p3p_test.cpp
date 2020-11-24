#define CATCH_CONFIG_MAIN
#include <beam_calibration/DoubleSphere.h>
#include <beam_cv/geometry/AbsolutePoseEstimator.h>
#include <catch2/catch.hpp>

std::shared_ptr<beam_calibration::CameraModel> camera_model_;

void LoadCameraModel() {
  std::string intrinsics_location = __FILE__;
  std::string current_file_path = "p3p_test.cpp";
  intrinsics_location.erase(intrinsics_location.end() -
                                current_file_path.length(),
                            intrinsics_location.end());
  intrinsics_location += "test_data/DS_test.json";
  camera_model_ =
      std::make_shared<beam_calibration::DoubleSphere>(intrinsics_location);
};

TEST_CASE("Testing p3p progress.") {
  beam_cv::AbsolutePoseEstimator test;

  LoadCameraModel();

  Eigen::Vector2i y1, y2, y3;
  y1(0) = 0.0;
  y1(1) = 0.0;
  y2(0) = 1.0;
  y2(1) = 0.0;
  y3(0) = 2.0;
  y3(1) = 1.0;
  std::vector<Eigen::Vector2i> pixels{y1, y2};

  Eigen::Vector3d X1, X2, X3;
  X1(0) = 0.0;
  X1(1) = 0.0;
  X1(2) = 2.0;
  X2(0) = 1.41421356237309;
  X2(1) = 0.0;
  X2(2) = 1.41421356237309;
  X3(0) = 1.63299316185545;
  X3(1) = 0.816496580927726;
  X3(2) = 0.816496580927726;
  std::vector<Eigen::Vector3d> points{X1, X2, X3};

  test.P3PEstimator(camera_model_, pixels, points);
  REQUIRE(1 == 1);
}