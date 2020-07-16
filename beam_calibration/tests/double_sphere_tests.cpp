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

Eigen::Vector2d ProjectPointPrecise(const Eigen::Vector3d& point){
  Eigen::VectorXd intrinsics = camera_model_->GetIntrinsics();
  double fx_ = intrinsics[0];
  double fy_ = intrinsics[1];
  double cx_ = intrinsics[2];
  double cy_ = intrinsics[3];
  double eps_ = intrinsics[4];
  double alpha_ = intrinsics[5];

  double d1 =
      sqrt(point[0] * point[0] + point[1] * point[1] + point[2] * point[2]);
  double d2 = sqrt(point[0] * point[0] + point[1] * point[1] +
                   (eps_ * d1 + point[2]) * (eps_ * d1 + point[2]));
  Eigen::Vector2d point_projected;
  point_projected[0] =
      fx_ * point[0] /
      (alpha_ * d2 + (1 - alpha_) * (eps_ * d1 + point[2])) + cx_;
  point_projected[1] =
      fy_ * point[1] /
      (alpha_ * d2 + (1 - alpha_) * (eps_ * d1 + point[2])) + cy_;
  return point_projected;
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

  for (Eigen::Vector3d point : points) {
    opt<Eigen::Vector2i> pixel = camera_model_->ProjectPoint(point);
    if (!pixel.has_value()) { continue; }
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

  // create random test points
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

    // calculate numerical jacobian
    Eigen::MatrixXd J_numerical(2, 3);
    for (int i = 0; i < 3; i++) {
      Eigen::Vector3d perturbation(0, 0, 0);
      perturbation[i] = eps;
      Eigen::Vector3d point_left_pert = point - perturbation;
      Eigen::Vector3d point_right_pert = point + perturbation;
      Eigen::Vector2d pixel_left_pert = ProjectPointPrecise(point_left_pert);
      Eigen::Vector2d pixel_right_pert = ProjectPointPrecise(point_right_pert);
      J_numerical(0, i) = (pixel_right_pert[0] - pixel_left_pert[0]) / (2*eps);
      J_numerical(1, i) = (pixel_right_pert[1] - pixel_left_pert[1]) / (2*eps);
    }
    REQUIRE(beam::RoundMatrix(J_numerical, 4) ==
            beam::RoundMatrix(J_analytical, 4));
  }
}