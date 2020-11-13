#define CATCH_CONFIG_MAIN

#include <boost/filesystem.hpp>
#include <catch2/catch.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <beam_calibration/CameraModels.h>
#include <beam_calibration/ConvertCameraModel.h>

bool save_images_ = true;
bool run_ladybug_test_ = true;
std::string save_path_ = "/tmp/convert_camera_models_tests/";

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

std::shared_ptr<beam_calibration::CameraModel>
    LoadRadtanModel(const std::string& config) {
  std::string intrinsics_location = GetDataPath(config);
  std::shared_ptr<beam_calibration::CameraModel> camera_model =
      std::make_shared<beam_calibration::Radtan>(intrinsics_location);
  return camera_model;
}

void SaveImage(const std::string& image_name, const cv::Mat& image) {
  if (!save_images_) { return; }
  if (!boost::filesystem::exists(save_path_)) {
    boost::filesystem::create_directory(save_path_);
  }
  std::string full_name = save_path_ + image_name;
  std::cout << "Saving image: " << full_name << "\n";
  cv::imwrite(full_name, image);
}

/*
TEST_CASE("Test converting a Radtan Image to the same model") {
  // load model that created the image
  std::shared_ptr<beam_calibration::CameraModel> source_model =
      LoadRadtanModel("camera_model_conversion_test_intrinsics.json");

  // create copy
  std::shared_ptr<beam_calibration::CameraModel> output_model =
      LoadRadtanModel("camera_model_conversion_test_intrinsics.json");

  // load original image
  std::string image_path = GetDataPath("image.png");
  cv::Mat image_in = cv::imread(image_path, cv::IMREAD_COLOR);

  // crop image to speed up test
  int x = static_cast<int>(source_model->GetWidth() / 2);
  int y = static_cast<int>(source_model->GetHeight() / 2);
  int width = 50;
  int height = 50;
  cv::Rect ROI(x, y, width, height);
  cv::Mat source_image = image_in(ROI);

  // convert image
  beam_calibration::ConvertCameraModel converter(source_model, width, height,
                                                 output_model);
  cv::Mat output_image = converter.ConvertImage<cv::Vec3b>(source_image);

  REQUIRE(source_model->GetWidth() == output_model->GetWidth());
  REQUIRE(source_model->GetHeight() == output_model->GetHeight());
  REQUIRE(source_image.rows == output_image.rows);
  REQUIRE(source_image.cols == output_image.cols);

  // check every n pixel and check that more than more than P percent pass
  // we expect some artifacts due to pixel integer rounding
  // Errors are often at borders so we skip border pixels
  int n = 5;
  double P = 0.99;
  int num_correct = 0;
  int num_total = 0;
  for (int i = 1; i < height - 1; i += n) {
    for (int j = 1; j < width - 1; j += n) {
      if (source_image.at<cv::Vec3b>(i, j) ==
          output_image.at<cv::Vec3b>(i, j)) {
        num_correct++;
      }
      num_total++;
    }
  }
  double percent_correct =
      static_cast<double>(num_correct) / static_cast<double>(num_total);
  REQUIRE(percent_correct > P);

  SaveImage("test_case_1_image_original.png", source_image);
  SaveImage("test_case_1_image_new.png", output_image);
}
*/

TEST_CASE("Test distorting and undistoring a radtan simulation image") {
  // load model that created the image
  std::shared_ptr<beam_calibration::CameraModel> source_model =
      LoadRadtanModel("camera_model_conversion_test_intrinsics.json");

  // create distorted version of this model
  std::shared_ptr<beam_calibration::CameraModel> distorted_model =
      LoadRadtanModel("camera_model_conversion_test_intrinsics_distorted.json");

  // load original image
  std::string image_path = GetDataPath("image.png");
  cv::Mat image_read = cv::imread(image_path, cv::IMREAD_COLOR);

  // crop image to speed up test
  // int x = static_cast<int>(source_model->GetWidth() / 2);
  // int y = static_cast<int>(source_model->GetHeight() / 2);
  // int width = 50;
  // int height = 50;
  // cv::Rect ROI(x, y, width, height);
  // cv::Mat source_image = image_in(ROI);
  int width = static_cast<int>(source_model->GetWidth() / 1);
  int height = static_cast<int>(source_model->GetHeight() / 1);
  cv::Mat source_image = image_read;

  // distort image
  beam_calibration::ConvertCameraModel no_distortion_to_distorted(
      source_model, width, height, distorted_model);
  cv::Mat distorted_image =
      no_distortion_to_distorted.ConvertImage<cv::Vec3b>(source_image);

  // undistort image
  beam_calibration::ConvertCameraModel distorted_to_no_distortion(
      distorted_model, width, height);
  cv::Mat undistorted_image =
      distorted_to_no_distortion.ConvertImage<cv::Vec3b>(distorted_image);

  // save images if bool is set
  SaveImage("test_case_2_image_original.png", source_image);
  SaveImage("test_case_2_image_distorted.png", distorted_image);
  SaveImage("test_case_2_image_undistorted.png", undistorted_image);

  // check every n pixel and check that more than more than P percent pass
  // we expect some artifacts due to pixel integer rounding.
  // Errors are often at borders so we skip border pixels
  int n = 5;
  double P = 0.98;
  int num_correct = 0;
  int num_total = 0;
  for (int i = 1; i < height - 1; i += n) {
    for (int j = 1; j < width - 1; j += n) {
      if (source_image.at<cv::Vec3b>(i, j) ==
          undistorted_image.at<cv::Vec3b>(i, j)) {
        num_correct++;
      }
      num_total++;
    }
  }

  double percent_correct =
      static_cast<double>(num_correct) / static_cast<double>(num_total);
  REQUIRE(percent_correct > P);
}

TEST_CASE("Test undistorting a ladybug image") {
  if (!run_ladybug_test_) { REQUIRE(true); }

  std::shared_ptr<beam_calibration::CameraModel> source_model =
      LoadLadybugCameraModel();

  Eigen::Vector3d point(0,0,1);
  Eigen::Vector2d pixel = source_model->ProjectPointPrecise(point).value();
  std::cout << "Point: \n" << point << "\n";
  std::cout << "Projected Pixel: \n" << pixel << "\n";

  std::string image_path = GetDataPath("ladybug_camera_3_image2.png");
  cv::Mat source_image = cv::imread(image_path, cv::IMREAD_COLOR);

  std::shared_ptr<beam_calibration::CameraModel> distorted_model =
      LoadRadtanModel("camera_model_conversion_test_intrinsics_ladybug.json");

  beam_calibration::ConvertCameraModel converter(
      source_model, source_model->GetWidth(), source_model->GetHeight(),
      distorted_model);

  cv::Mat output_image;
  REQUIRE_NOTHROW(output_image =
                      converter.ConvertImage<cv::Vec3b>(source_image));
  SaveImage("test_case_3_image_original.png", source_image);
  SaveImage("test_case_3_image_undistorted.png", output_image);
}