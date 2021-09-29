#define CATCH_CONFIG_MAIN

#include <boost/filesystem.hpp>
#include <catch2/catch.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <beam_calibration/CameraModels.h>
#include <beam_calibration/ConvertCameraModel.h>

bool save_images_ = false;
std::string save_path_ = "/tmp/convert_camera_models_tests/";

// These two tests need to be visually assessed for accuracy so they are off by
// default so that it is not run by travis
bool run_ladybug_test_ = false;
bool run_kb_test_ = false;

std::string GetDataPath(const std::string& filename) {
  std::string file_location = __FILE__;
  std::string current_file_path = "convert_camera_models_tests.cpp";
  file_location.erase(file_location.end() - current_file_path.length(),
                      file_location.end());
  file_location += "test_data/" + filename;
  return file_location;
}

// std::shared_ptr<beam_calibration::CameraModel> LoadLadybugCameraModel() {
//   std::string intrinsics_location = GetDataPath("ladybug.conf");
//   std::shared_ptr<beam_calibration::CameraModel> camera_model =
//       std::make_shared<beam_calibration::Ladybug>(intrinsics_location);
//   return camera_model;
// }

std::shared_ptr<beam_calibration::CameraModel> LoadKBCameraModel() {
  std::string intrinsics_location = GetDataPath("KB_test.json");
  std::shared_ptr<beam_calibration::CameraModel> camera_model =
      std::make_shared<beam_calibration::KannalaBrandt>(intrinsics_location);
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
  Eigen::Vector2i dims(height, width);

  // convert image
  beam_calibration::ConvertCameraModel converter(source_model, dims, dims,
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
  double P = 0.93;
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

TEST_CASE("Test distorting and undistoring a radtan simulation image") {
  // load model that created the image
  std::shared_ptr<beam_calibration::CameraModel> source_model =
      LoadRadtanModel("camera_model_conversion_test_intrinsics.json");

  // create distorted version of this model
  std::shared_ptr<beam_calibration::CameraModel> distorted_model =
      LoadRadtanModel("camera_model_conversion_test_intrinsics_distorted.json");

  // load original image
  std::string image_path = GetDataPath("image.png");
  cv::Mat source_image = cv::imread(image_path, cv::IMREAD_COLOR);

  // get dimensions
  Eigen::Vector2i src_image_dims(source_image.rows, source_image.cols);
  Eigen::Vector2i dst_image_dims_same(source_image.rows, source_image.cols);
  Eigen::Vector2i dst_image_dims_smaller(source_image.rows * 0.8,
                                         source_image.cols * 0.8);
  Eigen::Vector2i dst_image_dims_larger(source_image.rows * 1.2,
                                        source_image.cols * 1.2);

  // distort image
  beam_calibration::ConvertCameraModel no_distortion_to_distorted(
      source_model, src_image_dims, dst_image_dims_same, distorted_model);
  cv::Mat distorted_image =
      no_distortion_to_distorted.ConvertImage<cv::Vec3b>(source_image);

  // undistort image
  beam_calibration::ConvertCameraModel distorted_to_no_distortion(
      distorted_model, src_image_dims, dst_image_dims_same);
  cv::Mat undistorted_image =
      distorted_to_no_distortion.ConvertImage<cv::Vec3b>(distorted_image);

  // test going from full size image to smaller image (using undistort wrapper)
  beam_calibration::UndistortImages distorted_to_no_distortion_smaller(
      distorted_model, src_image_dims, dst_image_dims_smaller);
  cv::Mat undistorted_image_smaller =
      distorted_to_no_distortion_smaller.ConvertImage<cv::Vec3b>(
          distorted_image);

  // test going from full size image to smaller image
  beam_calibration::ConvertCameraModel distorted_to_no_distortion_larger(
      distorted_model, src_image_dims, dst_image_dims_larger);
  cv::Mat undistorted_image_larger =
      distorted_to_no_distortion_larger.ConvertImage<cv::Vec3b>(
          distorted_image);

  // test going from cropped image to same size
  int width = static_cast<int>(distorted_image.cols * 0.75);
  int height = static_cast<int>(distorted_image.rows * 0.75);
  int x = static_cast<int>((distorted_image.cols - width) / 2);
  int y = static_cast<int>((distorted_image.rows - height) / 2);
  cv::Rect ROI(x, y, width, height);
  cv::Mat cropped_image_distorted = distorted_image(ROI);
  beam_calibration::ConvertCameraModel distorted_to_no_distortion_cropped(
      distorted_model,
      Eigen::Vector2i(cropped_image_distorted.rows,
                      cropped_image_distorted.cols),
      Eigen::Vector2i(cropped_image_distorted.rows,
                      cropped_image_distorted.cols));
  cv::Mat cropped_image_undistorted =
      distorted_to_no_distortion_cropped.ConvertImage<cv::Vec3b>(
          cropped_image_distorted);

  // save images if bool is set
  SaveImage("test_case_2_image_original.png", source_image);
  SaveImage("test_case_2_image_distorted.png", distorted_image);
  SaveImage("test_case_2_image_undistorted.png", undistorted_image);
  SaveImage("test_case_2_image_undistorted_smaller.png",
            undistorted_image_smaller);
  SaveImage("test_case_2_image_undistorted_larger.png",
            undistorted_image_larger);
  SaveImage("test_case_2_cropped_image_distorted.png", cropped_image_distorted);
  SaveImage("test_case_2_cropped_image_undistorted.png",
            cropped_image_undistorted);

  // Check the output image dimensions
  REQUIRE(source_image.cols == distorted_image.cols);
  REQUIRE(source_image.rows == distorted_image.rows);
  REQUIRE(source_image.cols == undistorted_image.cols);
  REQUIRE(source_image.rows == undistorted_image.rows);
  REQUIRE(static_cast<int>(source_image.cols * 1.2) ==
          undistorted_image_larger.cols);
  REQUIRE(static_cast<int>(source_image.rows * 1.2) ==
          undistorted_image_larger.rows);
  REQUIRE(static_cast<int>(source_image.cols * 0.8) ==
          undistorted_image_smaller.cols);
  REQUIRE(static_cast<int>(source_image.rows * 0.8) ==
          undistorted_image_smaller.rows);
  REQUIRE(static_cast<int>(source_image.cols * 0.75) ==
          cropped_image_undistorted.cols);
  REQUIRE(static_cast<int>(source_image.rows * 0.75) ==
          cropped_image_undistorted.rows);

  // Compare source image to undistorted image - they should be approx the same.
  // Check every n pixel and check that more than more than P percent pass
  // we expect some artifacts due to pixel integer rounding.
  // Errors are often at borders so we skip border pixels
  int n = 5;
  double P = 0.97;
  int num_correct = 0;
  int num_total = 0;
  for (uint16_t i = 1; i < source_model->GetHeight() - 1; i += n) {
    for (uint16_t j = 1; j < source_model->GetWidth() - 1; j += n) {
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

  // Compare source image to undistorted cropped image - they should be approx
  // the same. Check every n pixel and check that more than more than P percent
  // pass we expect some artifacts due to pixel integer rounding. Errors are
  // often at borders so we skip border pixels
  cv::Mat tmp_image = undistorted_image(ROI);
  num_correct = 0;
  num_total = 0;
  for (uint16_t i = 1; i < cropped_image_undistorted.rows - 1; i += n) {
    for (uint16_t j = 1; j < cropped_image_undistorted.cols - 1; j += n) {
      if (tmp_image.at<cv::Vec3b>(i, j) ==
          cropped_image_undistorted.at<cv::Vec3b>(i, j)) {
        num_correct++;
      }
      num_total++;
    }
  }
  percent_correct =
      static_cast<double>(num_correct) / static_cast<double>(num_total);
  REQUIRE(percent_correct > P);
}

// This commented out because we no longer require the ladybug model to be built
// TEST_CASE("Test undistorting a ladybug image") {
//   if (!run_ladybug_test_) {
//     REQUIRE(true);
//     return;
//   }

//   std::shared_ptr<beam_calibration::CameraModel> source_model =
//       LoadLadybugCameraModel();

//   std::string image_path = GetDataPath("ladybug_undistort.png");
//   cv::Mat source_image = cv::imread(image_path, cv::IMREAD_COLOR);

//   Eigen::Vector2i image_dims(source_model->GetHeight(),
//                              source_model->GetWidth());

//   beam_calibration::ConvertCameraModel converter(source_model, image_dims,
//                                                  image_dims);

//   cv::Mat upsampled_image = converter.UpsampleImage(source_image);

//   cv::Mat output_image;
//   REQUIRE_NOTHROW(output_image =
//                       converter.ConvertImage<cv::Vec3b>(upsampled_image));

//   cv::Mat downsampled_image = converter.DownsampleImage(
//       output_image, Eigen::Vector2i(source_image.rows, source_image.cols));

//   SaveImage("test_case_3_image_original.png", source_image);
//   SaveImage("test_case_3_image_upsampled.png", upsampled_image);
//   SaveImage("test_case_3_image_undistorted_downsampled.png",
//   downsampled_image); SaveImage("test_case_3_image_undistorted.png",
//   output_image);

//   // Check the output image dimensions
//   REQUIRE(source_image.cols == downsampled_image.cols);
//   REQUIRE(source_image.rows == downsampled_image.rows);
//   REQUIRE(upsampled_image.cols == output_image.cols);
//   REQUIRE(upsampled_image.rows == output_image.rows);
//   REQUIRE(upsampled_image.cols == source_model->GetWidth());
//   REQUIRE(upsampled_image.rows == source_model->GetHeight());
// }

TEST_CASE("Test undistorting a kannala brandt image") {
  if (!run_kb_test_) {
    REQUIRE(true);
    return;
  }
  std::shared_ptr<beam_calibration::CameraModel> source_model =
      LoadKBCameraModel();

  std::string image_path = GetDataPath("kb_undistort.png");

  cv::Mat source_image = cv::imread(image_path, cv::IMREAD_COLOR);

  Eigen::Vector2i image_dims(source_model->GetHeight(),
                             source_model->GetWidth());

  beam_calibration::ConvertCameraModel converter(source_model, image_dims,
                                                 image_dims);

  cv::Mat upsampled_image = converter.UpsampleImage(source_image);

  cv::Mat output_image;
  REQUIRE_NOTHROW(output_image =
                      converter.ConvertImage<cv::Vec3b>(upsampled_image));
  SaveImage("test_case_4_image_original.png", source_image);
  SaveImage("test_case_4_image_upsampled.png", upsampled_image);
  SaveImage("test_case_4_image_undistorted.png", output_image);
}