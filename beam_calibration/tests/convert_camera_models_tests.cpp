#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <beam_calibration/CameraModels.h>
#include <beam_calibration/ConvertCameraModel.h>

std::string GetDataPath(const std::string& filename){
  std::string file_location = __FILE__;
  std::string current_file_path = "convert_camera_models_tests.cpp";
  file_location.erase(file_location.end() -
                                current_file_path.length(),
                            file_location.end());
  file_location += filename;
  return file_location;
}

std::shared_ptr<beam_calibration::CameraModel> LoadLadybugCameraModel() {
  std::string intrinsics_location = GetDataPath("ladybug.conf");
  std::shared_ptr<beam_calibration::CameraModel> camera_model =
      std::make_shared<beam_calibration::Ladybug>(intrinsics_location);
  return camera_model;
}

TEST_CASE("Test undistorting a ladybug image") {
  std::shared_ptr<beam_calibration::CameraModel> source_model =
      LoadLadybugCameraModel();
  std::string image_path = GetDataPath("ladybug_camera_3_image_2.png");
  cv::Mat source_image = cv::imread(image_path, cv::IMREAD_COLOR);
  beam_calibration::ConvertCameraModel converter(source_model, source_model->GetWidth(), source_model->GetHeight());
  cv::Mat output_image = converter.ConvertImage<cv::Vec3i>(source_image);
  std::string output_filename = "/home/nick/test_image.png";
  cv::imwrite(output_filename, output_image);
}
