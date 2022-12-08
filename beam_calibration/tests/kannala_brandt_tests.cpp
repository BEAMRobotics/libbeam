#define CATCH_CONFIG_MAIN

#include <beam_calibration/KannalaBrandt.h>

#include <beam_calibration/CameraModel.h>
#include <beam_utils/math.h>
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
  std::vector<Eigen::Vector3d, beam::AlignVec3d> points;
  for (int i = 0; i < numRandomCases1; i++) {
    double x = fRand(min_x, max_x);
    double y = fRand(min_y, max_y);
    double z = fRand(min_z, max_z);
    points.push_back(Eigen::Vector3d(x, y, z));
  }

  bool in_image = false;
  for (const Eigen::Vector3d point : points) {
    Eigen::Vector2d pixel;
    if (camera_model_->ProjectPoint(point, pixel, in_image)) { continue; }
    Eigen::Vector3d back_projected_ray;
    bool success =
        camera_model_->BackProject(pixel.cast<int>(), back_projected_ray);
    REQUIRE(success);
    if (success) {
      Eigen::Vector3d back_projected_point = point.norm() * back_projected_ray;
      Eigen::Vector2d back_projected_pixel;
      REQUIRE(camera_model_->ProjectPoint(back_projected_point,
                                          back_projected_pixel, in_image));
      REQUIRE(std::abs(pixel[0] - back_projected_pixel[0]) < 2);
      REQUIRE(std::abs(pixel[1] - back_projected_pixel[1]) < 2);
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

  std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
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
    bool in_domain = camera_model_->BackProject(pixel, ray);
    if (!in_domain) { continue; }
    Eigen::Vector3d point = ray * fRand(min_d, max_d);
    Eigen::Vector2d projected_pixel;
    in_domain = camera_model_->ProjectPoint(point, projected_pixel, in_image);
    if (!in_domain || !in_image) { continue; }
    REQUIRE(std::abs(pixel[0] - projected_pixel[0]) < 2);
    REQUIRE(std::abs(pixel[1] - projected_pixel[1]) < 2);
  }
}

TEST_CASE("Test projection and back project with invalid points/pixels") {
  LoadCameraModel();

  // create random test points that are outside projection domain
  int numRandomCases3 = 5;
  double min_x_b = -2;
  double max_x_b = 2;
  double min_y_b = -2;
  double max_y_b = 2;
  double z_b = 0;
  bool in_image = false;
  for (int i = 0; i < numRandomCases3; i++) {
    double x_b = fRand(min_x_b, max_x_b);
    double y_b = fRand(min_y_b, max_y_b);
    Eigen::Vector3d point_b(x_b, y_b, z_b);
    Eigen::Vector2d pixel;
    REQUIRE(!camera_model_->ProjectPoint(point_b, pixel, in_image));
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
  std::vector<Eigen::Vector3d, beam::AlignVec3d> points;
  for (int i = 0; i < numRandomCases1; i++) {
    double x = fRand(min_x, max_x);
    double y = fRand(min_y, max_y);
    double z = fRand(min_z, max_z);
    points.push_back(Eigen::Vector3d(x, y, z));
  }

  double eps = std::sqrt(1e-8);
  bool in_image = false;
  for (Eigen::Vector3d point : points) {
    // calculate analytical jacobian (from camera model)
    std::shared_ptr<Eigen::MatrixXd> J_analytical =
        std::make_shared<Eigen::MatrixXd>(2, 3);
    Eigen::Vector2d tmp;
    REQUIRE(camera_model_->ProjectPoint(point, tmp, in_image, J_analytical));

    // calculate numerical jacobian
    Eigen::MatrixXd J_numerical(2, 3);
    for (int i = 0; i < 3; i++) {
      Eigen::Vector3d perturbation(0, 0, 0);
      perturbation[i] = eps;
      Eigen::Vector3d point_pert = point + perturbation;
      Eigen::Vector2d pixel;
      camera_model_->ProjectPoint(point, pixel, in_image);
      Eigen::Vector2d pixel_pert;
      camera_model_->ProjectPoint(point_pert, pixel_pert, in_image);
      J_numerical(0, i) = (pixel_pert[0] - pixel[0]) / eps;
      J_numerical(1, i) = (pixel_pert[1] - pixel[1]) / eps;
    }
    REQUIRE(J_numerical.isApprox(*J_analytical, 1e-4));
  }
}
