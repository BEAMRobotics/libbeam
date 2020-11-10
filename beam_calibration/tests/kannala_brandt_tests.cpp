#define CATCH_CONFIG_MAIN

#include <beam_calibration/KannalaBrandt.h>

#include <beam_calibration/CameraModel.h>
#include <beam_utils/math.hpp>
#include <catch2/catch.hpp>

std::unique_ptr<beam_calibration::CameraModel> camera_model_;

double fRand(double fMin, double fMax) {
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

void LoadCameraModel() {
  std::string intrinsics_location = __FILE__;
  std::string current_file_path = "kannala_brandt_tests.cpp";
  intrinsics_location.erase(intrinsics_location.end() -
                                current_file_path.length(),
                            intrinsics_location.end());
  intrinsics_location += "test_data/KB_test.json";
  camera_model_ =
      std::make_unique<beam_calibration::KannalaBrandt>(intrinsics_location);
}

TEST_CASE("Test create method") {
  std::string intrinsics_location = __FILE__;
  std::string current_file_path = "kannala_brandt_tests.cpp";
  intrinsics_location.erase(intrinsics_location.end() -
                                current_file_path.length(),
                            intrinsics_location.end());
  intrinsics_location += "test_data/KB_test.json";
  std::shared_ptr<beam_calibration::CameraModel> cm =
      beam_calibration::CameraModel::Create(intrinsics_location);
  REQUIRE(cm->GetType() == beam_calibration::CameraType::KANNALABRANDT);
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

  bool outside_domain = false;

  for (Eigen::Vector3d point : points) {
    opt<Eigen::Vector2i> pixel = camera_model_->ProjectPoint(point);
    opt<Eigen::Vector2i> pixel_b = camera_model_->ProjectPoint(point, outside_domain);
    if (!pixel.has_value() || !pixel_b.has_value()) { continue; }
    REQUIRE((pixel.value()[0] - pixel_b.value()[0]) == 0);
    REQUIRE((pixel.value()[1] - pixel_b.value()[1]) == 0);
    opt<Eigen::Vector3d> back_projected_ray =
        camera_model_->BackProject(pixel.value());
    REQUIRE(back_projected_ray.has_value());
    if (back_projected_ray.has_value()) {
      Eigen::Vector3d back_projected_point =
          point.norm() * back_projected_ray.value();
      opt<Eigen::Vector2i> back_projected_pixel =
          camera_model_->ProjectPoint(back_projected_point);
      REQUIRE(back_projected_pixel.has_value());
      REQUIRE(std::abs(pixel.value()[0] - back_projected_pixel.value()[0]) < 2);
      REQUIRE(std::abs(pixel.value()[1] - back_projected_pixel.value()[1]) < 2);
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

  double min_d = 1;
  double max_d = 10;
  for (Eigen::Vector2i pixel : pixels) {
    opt<Eigen::Vector3d> ray = camera_model_->BackProject(pixel);
    REQUIRE(ray.has_value());
    if (!ray.has_value()) { continue; }
    Eigen::Vector3d point = ray.value() * fRand(min_d, max_d);
    opt<Eigen::Vector2i> projected_pixel = camera_model_->ProjectPoint(point);
    REQUIRE(projected_pixel.has_value());
    if (!projected_pixel.has_value()) { continue; }
    REQUIRE(std::abs(pixel[0] - projected_pixel.value()[0]) < 2);
    REQUIRE(std::abs(pixel[1] - projected_pixel.value()[1]) < 2);
  }
}

TEST_CASE("Test projection and back project with invalid points/pixels") {
  LoadCameraModel();
  uint32_t w = camera_model_->GetWidth();
  uint32_t h = camera_model_->GetHeight();

  // create random test points
  uint32_t numRandomCases1 = 20;
  double min_u = -w;
  double max_u = 2 * w;
  double min_v = -h;
  double max_v = 2 * h;

  std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>>
      pixels;
  while (pixels.size() < numRandomCases1) {
    double u = fRand(min_u, max_u);
    double v = fRand(min_v, max_v);
    Eigen::Vector2i pixel((int)u, (int)v);
    if (camera_model_->PixelInImage(pixel)) { continue; }
    pixels.push_back(pixel);
  }

  for (Eigen::Vector2i pixel : pixels) {
    opt<Eigen::Vector3d> ray = camera_model_->BackProject(pixel);
    REQUIRE(!ray.has_value());
  }

  // create random test points that result in projections out of frame
  int numRandomCases2 = 30;
  double min_x = -2;
  double max_x = 2;
  double min_y = -2;
  double max_y = 2;
  double min_z = -2;
  double max_z = -10;
  for (int i = 0; i < numRandomCases2; i++) {
    double x = fRand(min_x, max_x);
    double y = fRand(min_y, max_y);
    double z = fRand(min_z, max_z);
    Eigen::Vector3d point(x, y, z);
    opt<Eigen::Vector2i> pixel = camera_model_->ProjectPoint(point);
    REQUIRE(!pixel.has_value());
  }

  bool outside_domain = false;

  //create random test points that are outside projection domain
  int numRandomCases3 = 5;
  double min_x_b = -2;
  double max_x_b = 2;
  double min_y_b = -2;
  double max_y_b = 2;
  double z_b = 0;
  for (int i = 0; i < numRandomCases3; i++) {
    double x_b = fRand(min_x_b, max_x_b);
    double y_b = fRand(min_y_b, max_y_b);
    Eigen::Vector3d point_b(x_b, y_b, z_b);
    opt<Eigen::Vector2i> pixel = camera_model_->ProjectPoint(point_b, outside_domain);
    REQUIRE(!pixel.has_value());
    REQUIRE(outside_domain == true);
  }

}


TEST_CASE("Test jacobian") {
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

  double eps = std::sqrt(1e-10);
  for (Eigen::Vector3d point : points) {
    // calculate analytical jacobian (from camera model)
    Eigen::MatrixXd J_analytical(2, 3);
    opt<Eigen::Vector2i> tmp = camera_model_->ProjectPoint(point, J_analytical);
    REQUIRE(tmp.has_value());

    // calculate numerical jacobian
    Eigen::MatrixXd J_numerical(2, 3);
    for (int i = 0; i < 3; i++) {
      Eigen::Vector3d perturbation(0, 0, 0);
      perturbation[i] = eps;
      Eigen::Vector3d point_pert = point + perturbation;
      opt<Eigen::Vector2d> pixel = camera_model_->ProjectPointPrecise(point);
      opt<Eigen::Vector2d> pixel_pert =
          camera_model_->ProjectPointPrecise(point_pert);
      J_numerical(0, i) = (pixel_pert.value()[0] - pixel.value()[0]) / eps;
      J_numerical(1, i) = (pixel_pert.value()[1] - pixel.value()[1]) / eps;
    }
    REQUIRE(J_numerical.isApprox(J_analytical, 0.001));
  }
}
