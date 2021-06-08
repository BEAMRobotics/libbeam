#define CATCH_CONFIG_MAIN

#include <beam_calibration/Ladybug.h>
#include <beam_utils/math.h>
#include <catch2/catch.hpp>

std::unique_ptr<beam_calibration::CameraModel> camera_model_;

double fRand(double fMin, double fMax) {
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

void LoadCameraModel() {
  std::string intrinsics_location = __FILE__;
  std::string current_file_path = "ladybug_tests.cpp";
  intrinsics_location.erase(intrinsics_location.end() -
                                current_file_path.length(),
                            intrinsics_location.end());
  intrinsics_location += "test_data/ladybug.conf";
  camera_model_ =
      std::make_unique<beam_calibration::Ladybug>(intrinsics_location);
}

TEST_CASE("Test projection and back project with random points") {
  LoadCameraModel();

  // create random test points
  int numRandomCases1 = 30;
  double min_x = -2;
  double max_x = 2;
  double min_y = -2;
  double max_y = 2;
  double min_z = 2;
  double max_z = 10;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      points;
  for (int i = 0; i < numRandomCases1; i++) {
    double x = fRand(min_x, max_x);
    double y = fRand(min_y, max_y);
    double z = fRand(min_z, max_z);
    points.push_back(Eigen::Vector3d(x, y, z));
  }
  for (uint8_t id = 0; id < 6; id++) {
    camera_model_->SetCameraID(id);
    bool in_image = false;
    for (const Eigen::Vector3d point : points) {
      Eigen::Vector2d pixel;
      if (camera_model_->ProjectPoint(point, pixel, in_image)) { continue; }
      Eigen::Vector3d back_projected_ray;
      bool success =
          camera_model_->BackProject(pixel.cast<int>(), back_projected_ray);
      REQUIRE(success);
      if (success) {
        Eigen::Vector3d back_projected_point =
            point.norm() * back_projected_ray;
        Eigen::Vector2d back_projected_pixel;
        REQUIRE(camera_model_->ProjectPoint(back_projected_point,
                                            back_projected_pixel, in_image));
        REQUIRE(std::abs(pixel[0] - back_projected_pixel[0]) < 2);
        REQUIRE(std::abs(pixel[1] - back_projected_pixel[1]) < 2);
      }
    }
  }
}

TEST_CASE("Test projection and back project with random pixels") {
  LoadCameraModel();
  uint32_t w = camera_model_->GetWidth();
  uint32_t h = camera_model_->GetHeight();

  // create random test points
  int numRandomCases1 = 30;
  double min_u = 0;
  double max_u = w;
  double min_v = 0;
  double max_v = h;

  std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>>
      pixels;
  for (int i = 0; i < numRandomCases1; i++) {
    double u = fRand(min_u, max_u);
    double v = fRand(min_v, max_v);
    pixels.push_back(Eigen::Vector2i((int)u, (int)v));
  }
  bool in_image = false;
  double min_d = 4;
  double max_d = 10;
  for (Eigen::Vector2i pixel : pixels) {
    Eigen::Vector3d ray;
    bool pass = camera_model_->BackProject(pixel, ray);
    if (!pass) { continue; }
    Eigen::Vector3d point = ray * fRand(min_d, max_d);
    Eigen::Vector2d projected_pixel;
    pass = camera_model_->ProjectPoint(point, projected_pixel, in_image);
    if (!pass || !in_image) { continue; }
    REQUIRE(std::abs(pixel[0] - projected_pixel[0]) < 2);
    REQUIRE(std::abs(pixel[1] - projected_pixel[1]) < 2);
  }
}

TEST_CASE("Test Projection with invalid points") {
  LoadCameraModel();

  bool in_image = false;

  // create random test points that result in projections out of frame (outside
  // projection function domain)
  int numRandomCases1 = 30;
  double min_x = 8;
  double max_x = 10;
  double min_y = 8;
  double max_y = 10;
  double min_z = 1;
  double max_z = 2;
  for (int i = 0; i < numRandomCases1; i++) {
    double x = fRand(min_x, max_x);
    double y = fRand(min_y, max_y);
    double z = fRand(min_z, max_z);
    Eigen::Vector3d point(x, y, z);
    Eigen::Vector2d pixel;
    bool in_domain = camera_model_->ProjectPoint(point, pixel, in_image);
    REQUIRE(!in_domain);
  }
}
