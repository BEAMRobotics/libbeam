#define CATCH_CONFIG_MAIN

#include <beam_calibration/Radtan.h>
#include <beam_utils/math.h>
#include <catch2/catch.hpp>

std::unique_ptr<beam_calibration::CameraModel> camera_model_;

double fRand(double fMin, double fMax) {
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

void LoadCameraModel() {
  std::string intrinsics_location = __FILE__;
  std::string current_file_path = "radtan_tests.cpp";
  intrinsics_location.erase(intrinsics_location.end() -
                                current_file_path.length(),
                            intrinsics_location.end());
  intrinsics_location += "test_data/Radtan_test.json";
  camera_model_ =
      std::make_unique<beam_calibration::Radtan>(intrinsics_location);
}

TEST_CASE("Test create method") {
  std::string intrinsics_location = __FILE__;
  std::string current_file_path = "radtan_tests.cpp";
  intrinsics_location.erase(intrinsics_location.end() -
                                current_file_path.length(),
                            intrinsics_location.end());
  intrinsics_location += "test_data/Radtan_test.json";
  std::shared_ptr<beam_calibration::CameraModel> cm =
      beam_calibration::CameraModel::Create(intrinsics_location);
  REQUIRE(cm->GetType() == beam_calibration::CameraType::RADTAN);
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
    bool pass = camera_model_->BackProject(pixel, ray);
    REQUIRE(pass);
    if (!pass) { continue; }
    Eigen::Vector3d point = ray * fRand(min_d, max_d);
    Eigen::Vector2d projected_pixel;
    pass = camera_model_->ProjectPoint(point, projected_pixel, in_image);
    bool test = (in_image && pass) ? true : false;
    REQUIRE(test);
    if (!pass || !in_image) { continue; }
    REQUIRE(std::abs(pixel[0] - projected_pixel[0]) < 2);
    REQUIRE(std::abs(pixel[1] - projected_pixel[1]) < 2);
  }
}

TEST_CASE("Test projection and back project with invalid points/pixels") {
  LoadCameraModel();
  // create random test points that project out of image frame
  int numRandomCases2 = 30;
  double min_x = 6;
  double max_x = 8;
  double min_y = 6;
  double max_y = 8;
  double min_z = 0.01;
  double max_z = 0.1;
  bool in_image = false;
  for (int i = 0; i < numRandomCases2; i++) {
    double x = fRand(min_x, max_x);
    double y = fRand(min_y, max_y);
    double z = fRand(min_z, max_z);
    Eigen::Vector3d point(x, y, z);
    Eigen::Vector2d pixel;
    bool in_domain = camera_model_->ProjectPoint(point, pixel, in_image);
    REQUIRE(in_domain);
    REQUIRE(!in_image);
  }

  // create random test points that are out of projection domain
  int numRandomCases3 = 30;
  double min_x_b = 6;
  double max_x_b = 8;
  double min_y_b = 6;
  double max_y_b = 8;
  double min_z_b = -2;
  double max_z_b = -8;
  for (int i = 0; i < numRandomCases3; i++) {
    double x = fRand(min_x_b, max_x_b);
    double y = fRand(min_y_b, max_y_b);
    double z = fRand(min_z_b, max_z_b);
    Eigen::Vector3d point(x, y, z);
    Eigen::Vector2d pixel;
    bool in_domain = camera_model_->ProjectPoint(point, pixel, in_image);
    REQUIRE(!in_domain);
  }
}

TEST_CASE("Test jacobian") {
  LoadCameraModel();

  // create random test points
  int numRandomCases1 = 30;
  double min_x = -1;
  double max_x = 1;
  double min_y = -1;
  double max_y = 1;
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
    REQUIRE(J_numerical.isApprox(*J_analytical, 1e-2));
  }
}

TEST_CASE("Test safe projection") {
  std::string tmp = __FILE__;
  std::string current_file_path = "radtan_tests.cpp";
  tmp.erase(tmp.end() - current_file_path.length(), tmp.end());
  std::string intrinsics_location =
      tmp + "test_data/safe_projection_test_calib.json";
  std::string img_path = tmp += "test_data/safe_projection_test_img.jpg";

  auto camera_model =
      std::make_unique<beam_calibration::Radtan>(intrinsics_location);
  cv::Mat img = cv::imread(img_path, cv::IMREAD_COLOR);
  cv::Mat img_markedup = img.clone();

  // iterate through image pixels, back project, then reproject, and color all
  // images that land outside the visible range
  for (int u = 0; u < img.cols; u++) {
    for (int v = 0; v < img.rows; v++) {
      Eigen::Vector2i pixels(u, v);
      Eigen::Vector3d ray;
      camera_model->BackProject(pixels, ray);
      Eigen::Vector2d pixel_projected;
      bool valid = true;
      camera_model->ProjectPoint(ray, pixel_projected, valid);
      cv::Point p(u, v);
      if (!valid) { img_markedup.at<cv::Vec3b>(p) = cv::Vec3b(255, 0, 0); }
    }
  }
  cv::imwrite("/home/nick/test.jpg", img_markedup);
}