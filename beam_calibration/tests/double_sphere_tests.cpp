#define CATCH_CONFIG_MAIN

#include <beam_calibration/DoubleSphere.h>
#include <beam_utils/math.hpp>
#include <catch2/catch.hpp>

std::unique_ptr<beam_calibration::CameraModel> camera_model_;

double fRand(double fMin, double fMax) {
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

void LoadCameraModel() {
  std::string intrinsics_location = __FILE__;
  std::string current_file_path = "double_sphere_tests.cpp";
  intrinsics_location.erase(intrinsics_location.end() -
                                current_file_path.length(),
                            intrinsics_location.end());
  intrinsics_location += "test_data/DS_test.json";
  camera_model_ =
      std::make_unique<beam_calibration::DoubleSphere>(intrinsics_location);
}

TEST_CASE("Test projection and back project") {
  LoadCameraModel();
  
  // create random test points
  int numRandomCases1 = 10;
  double min_x = -2;
  double max_x = 2;
  double min_y = -2;
  double max_y = 2;
  double min_z = 2;
  double max_z = 10;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> points;
  for (int i = 0; i < numRandomCases1; i++) {
    double x = fRand(min_x, max_x);
    double y = fRand(min_y, max_y);
    double z = fRand(min_z, max_z);
    points.push_back(Eigen::Vector3d(x, y, z));
  }

  for (Eigen::Vector3d point : points){
    opt<Eigen::Vector2i> pixel = camera_model_->ProjectPoint(point);
    if(!pixel.has_value()){
      continue;
    }
    opt<Eigen::Vector3d> back_projected_ray = camera_model_->BackProject(pixel.value());
    REQUIRE(back_projected_ray.has_value() == true);
    if(back_projected_ray.has_value()){
      Eigen::Vector3d back_projected_point = point.norm() * back_projected_ray.value();
      REQUIRE(point[0] == Approx(back_projected_point[0]));
      REQUIRE(point[1] == Approx(back_projected_point[1]));
      REQUIRE(point[2] == Approx(back_projected_point[2]));
    }
  }
}
