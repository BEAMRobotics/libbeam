#define CATCH_CONFIG_MAIN

#include <fstream>
#include <iostream>

#include <beam_calibration/CameraModels.h>
#include <beam_cv/Utils.h>
#include <catch2/catch.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

std::vector<Eigen::Vector2i> ReadCircle(const std::string& filename) {
  std::string file_location = __FILE__;
  std::string current_file_path = "utils_test.cpp";
  file_location.erase(file_location.end() - current_file_path.length(),
                      file_location.end());
  file_location += "test_data/" + filename;
  // declare variables
  std::ifstream infile;
  std::string line;
  // open file
  infile.open(file_location);
  // extract poses
  std::vector<Eigen::Vector2i> circle;
  while (!infile.eof()) {
    // get timestamp k
    std::getline(infile, line, '\n');
    int u = std::stod(line);
    std::getline(infile, line, '\n');
    int v = std::stod(line);
    Eigen::Vector2i p{u, v};
    circle.push_back(p);
  }
  return circle;
}

TEST_CASE("Test connected components.") {
  std::string img_location = __FILE__;
  img_location.erase(img_location.end() - 20, img_location.end());
  img_location += "tests/test_data/crack.png";
  INFO(img_location);
  cv::Mat img = cv::imread(img_location, cv::IMREAD_GRAYSCALE);
  std::map<int, std::vector<cv::Point2i>> components =
      beam_cv::ConnectedComponents(img);
  REQUIRE(components.size() == 4);
}

TEST_CASE("Test K means segmentation.") {
  std::string img_location = __FILE__;
  img_location.erase(img_location.end() - 20, img_location.end());
  img_location += "tests/test_data/test.jpg";
  INFO(img_location);
  cv::Mat img = cv::imread(img_location, cv::IMREAD_COLOR);
  img = beam_cv::KMeans(img, 6);
  std::map<int, std::vector<cv::Point2i>> comp =
      beam_cv::ConnectedComponents(img);
  REQUIRE(comp.size() == 95846);
}

TEST_CASE("Test skeleton extraction.") {
  std::string img_location = __FILE__;
  img_location.erase(img_location.end() - 20, img_location.end());
  img_location += "tests/test_data/crack.png";
  INFO(img_location);
  cv::Mat img = cv::imread(img_location, cv::IMREAD_GRAYSCALE);
  std::vector<cv::Mat> components = beam_cv::SegmentComponents(img);
  cv::Mat crack = components[1];
  cv::Mat skel = beam_cv::ExtractSkeleton(crack);
  int num_white_full = 0, num_white_skel = 0;
  crack.forEach<uchar>([&](uchar& pix, const int* position) -> void {
    int skel_pix = skel.at<uchar>(position[0], position[1]);
    if (pix == 255) { num_white_full++; }
    if (skel_pix == 255) { num_white_skel++; }
  });
  REQUIRE((num_white_skel / num_white_full) < 0.6);
}

TEST_CASE("Test cluster removal.") {
  std::string img_location = __FILE__;
  img_location.erase(img_location.end() - 20, img_location.end());
  img_location += "tests/test_data/crack2.png";
  INFO(img_location);
  cv::Mat img = cv::imread(img_location, cv::IMREAD_GRAYSCALE);
  img = beam_cv::ExtractSkeleton(img);
  std::map<int, std::vector<cv::Point2i>> components =
      beam_cv::ConnectedComponents(img);
  REQUIRE(components.size() == 102);
  img = beam_cv::RemoveClusters(img, 20);
  components = beam_cv::ConnectedComponents(img);
  REQUIRE(components.size() == 82);
}

TEST_CASE("Circle extraction.") {
  std::string file_location = __FILE__;
  std::string current_file_path = "utils_test.cpp";
  file_location.erase(file_location.end() - current_file_path.length(),
                      file_location.end());
  file_location += "test_data/";

  // get kb camera model
  std::string intrinsics_location = file_location + "KB_test.json";
  std::shared_ptr<beam_calibration::CameraModel> cam_model_ =
      std::make_shared<beam_calibration::KannalaBrandt>(intrinsics_location);

  // get a circle around a pixel
  Eigen::Vector2i center(600, 600);
  std::vector<Eigen::Vector2i> circle = beam_cv::GetCircle(center, 10, 1.0);
  std::vector<Eigen::Vector2i> circle_gt = ReadCircle("circle.txt");
  for (size_t i = 0; i < circle.size(); i++) {
    REQUIRE(circle[i] == circle_gt[i]);
  }

  // undistort the circle
  std::vector<Eigen::Vector2i> circle_und;
  std::vector<Eigen::Vector2i> circle_und_gt =
      ReadCircle("circle_undistorted.txt");
  for (auto& p : circle) {
    circle_und.push_back(cam_model_->UndistortPixel(p));
  }
  for (size_t i = 0; i < circle_und.size(); i++) {
    REQUIRE(circle_und[i] == circle_und_gt[i]);
  }

  // draw results on image
  std::string image_path = file_location + "KB_test_img.png";
  cv::Mat source_image = cv::imread(image_path, cv::IMREAD_COLOR);

  for (auto& p : circle) {
    source_image.at<cv::Vec3b>(p[1], p[0]).val[0] = 0;
    source_image.at<cv::Vec3b>(p[1], p[0]).val[1] = 0;
    source_image.at<cv::Vec3b>(p[1], p[0]).val[2] = 255;
  }

  for (auto& p : circle_und) {
    source_image.at<cv::Vec3b>(p[1], p[0]).val[0] = 0;
    source_image.at<cv::Vec3b>(p[1], p[0]).val[1] = 255;
    source_image.at<cv::Vec3b>(p[1], p[0]).val[2] = 0;
  }
  cv::imwrite("/tmp/beam_cv_utils_tests/out.png", source_image);
}

TEST_CASE("Ellipse fitting.") {
  std::vector<Eigen::Vector2d> circle;
  circle.push_back(Eigen::Vector2d(-1, 0));
  circle.push_back(Eigen::Vector2d(1, 0));
  circle.push_back(Eigen::Vector2d(0, 1));
  circle.push_back(Eigen::Vector2d(0, -1));
  REQUIRE_THROWS(beam_cv::FitEllipse(circle));
  circle.push_back(Eigen::Vector2d(-0.5, -0.5));
  REQUIRE_NOTHROW(beam_cv::FitEllipse(circle));

  Eigen::Vector2i center(0, 0);
  std::vector<Eigen::Vector2i> circle_gt = beam_cv::GetCircle(center, 2, 0.5);
  std::vector<Eigen::Vector2d> circle2;
  for (auto& p : circle_gt) { circle2.push_back(Eigen::Vector2d(p[0], p[1])); }
  Eigen::Matrix2d ellipse = beam_cv::FitEllipse(circle2);
  Eigen::Matrix2d GT;
  GT << 4.71429, 0, 0, 4.71429;

  REQUIRE(ellipse.isApprox(GT, 0.1));
}