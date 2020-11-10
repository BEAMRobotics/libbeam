#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <beam_calibration/CameraModels.h>
#include <beam_calibration/ConvertCameraModel.h>

std::string GetDataPath(const std::string& filename) {
  std::string file_location = __FILE__;
  std::string current_file_path = "convert_camera_models_tests.cpp";
  file_location.erase(file_location.end() - current_file_path.length(),
                      file_location.end());
  file_location += "test_data/" + filename;
  return file_location;
}

std::shared_ptr<beam_calibration::CameraModel> LoadLadybugCameraModel() {
  std::string intrinsics_location = GetDataPath("ladybug.conf");
  std::shared_ptr<beam_calibration::CameraModel> camera_model =
      std::make_shared<beam_calibration::Ladybug>(intrinsics_location);
  return camera_model;
}

std::shared_ptr<beam_calibration::CameraModel> LoadRadtanModel() {
  std::string intrinsics_location =
      GetDataPath("camera_model_conversion_test_intrinsics.json");
  std::shared_ptr<beam_calibration::CameraModel> camera_model =
      std::make_shared<beam_calibration::Radtan>(intrinsics_location);
  return camera_model;
}

TEST_CASE("Test distorting and undistoring a radtan simulation image") {
  // load model that created the image
  std::shared_ptr<beam_calibration::CameraModel> source_model =
      LoadRadtanModel();

  // create distorted version of this model
  std::shared_ptr<beam_calibration::CameraModel> distorted_model =
      LoadRadtanModel();
  Eigen::VectorXd intrinsics_undistorted(8);
  Eigen::VectorXd intrinsics_distorted(8);
  intrinsics_undistorted = source_model->GetIntrinsics();
  intrinsics_distorted = intrinsics_undistorted;
  // intrinsics_distorted[4] = -0.2294924671994032;
  // intrinsics_distorted[5] = 0.18008566892263364;
  // intrinsics_distorted[6] = -0.0005326294604360527;
  // intrinsics_distorted[7] = -0.0004378797791316729;
  distorted_model->SetIntrinsics(intrinsics_distorted);

  // load original image
  std::string image_path = GetDataPath("image.png");
  cv::Mat source_image = cv::imread(image_path, cv::IMREAD_COLOR);

  // view images
  cv::imshow("original", source_image);
  cv::waitKey(0);
  cv::destroyAllWindows();

  // distort image
  std::cout << "TEST1\n";
  beam_calibration::ConvertCameraModel converter(
      source_model, source_model->GetWidth(), source_model->GetHeight(),
      distorted_model);

  std::cout << "TEST2\n";
  cv::Mat output_image = converter.ConvertImage<cv::Vec3i>(source_image);
  std::cout << "TEST3\n";

  // view images
  cv::imshow("original", source_image);
  // cv::imshow("new", output_image);
  cv::waitKey(0);
  cv::destroyAllWindows();

  // std::string output_filename = "/home/nick/test_image.png";
  // cv::imwrite(output_filename, output_image);
}

/*
TEST_CASE("Test undistorting a ladybug image") {
  std::shared_ptr<beam_calibration::CameraModel> source_model =
      LoadLadybugCameraModel();
  std::string image_path = GetDataPath("ladybug_camera_3_image2.png");
  cv::Mat source_image = cv::imread(image_path, cv::IMREAD_COLOR);
  std::cout << "TEST1\n";
  beam_calibration::ConvertCameraModel converter(
      source_model, source_model->GetWidth(), source_model->GetHeight());
  std::cout << "TEST2\n";
  cv::Mat output_image = converter.ConvertImage<cv::Vec3i>(source_image);
  std::cout << "TEST3\n";
  std::string output_filename = "/home/nick/test_image.png";
  std::cout << "TEST4\n";
  cv::imwrite(output_filename, output_image);
  std::cout << "TEST5\n";
}
*/