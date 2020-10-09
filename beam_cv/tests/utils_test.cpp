#define CATCH_CONFIG_MAIN
#include "beam_cv/Utils.h"
#include <catch2/catch.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

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