#define CATCH_CONFIG_MAIN
#include "beam_depth/Utils.h"
#include <catch2/catch.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

TEST_CASE("Test find closest.") {
  std::string img_location = __FILE__;
  img_location.erase(img_location.end() - 20, img_location.end());
  img_location += "tests/test_data/closest.png";
  cv::Mat img = cv::imread(img_location, cv::IMREAD_GRAYSCALE);
  Eigen::Vector2i sp1(0, 0);
  Eigen::Vector2i c1 = beam_depth::FindClosest(sp1, img);
  Eigen::Vector2i sp2(img.rows, img.cols);
  Eigen::Vector2i c2 = beam_depth::FindClosest(sp2, img);
  REQUIRE(c1[0] == 653);
  REQUIRE(c1[1] == 791);
  REQUIRE(c2[0] == 1021);
  REQUIRE(c2[1] == 1343);
}
