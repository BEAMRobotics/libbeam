#define CATCH_CONFIG_MAIN
#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>
#include <catch2/catch.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <torch/torch.h>

#include <beam_cv/descriptors/SuperPointDescriptor.h>
#include <beam_cv/detectors/SuperPointDetector.h>
#include <beam_cv/matchers/FLANNMatcher.h>

std::vector<Eigen::Vector2i> image_left_gt_matches_;
std::vector<Eigen::Vector2i> image_right_gt_matches_;
std::string model_file_path_;
bool show_results_{false};
bool save_results_{true};
std::string save_path_ = "/tmp/superpoint_tests/";
std::string data_path_;

void ReadMatches(std::string file, std::vector<Eigen::Vector2i>& matches1,
                 std::vector<Eigen::Vector2i>& matches2) {
  // declare variables
  std::ifstream infile;
  std::string line;
  // open file
  infile.open(file);
  // extract poses
  matches1.resize(0);
  matches2.resize(0);
  while (!infile.eof()) {
    // get timestamp k
    std::getline(infile, line, ',');
    int u1 = std::stod(line);
    std::getline(infile, line, ';');
    int v1 = std::stod(line);
    std::getline(infile, line, ',');
    int u2 = std::stod(line);
    std::getline(infile, line, '\n');
    int v2 = std::stod(line);
    Eigen::Vector2i p1{u1, v1};
    matches1.push_back(p1);
    Eigen::Vector2i p2{u2, v2};
    matches2.push_back(p2);
  }
}

cv::Mat GetImage(const std::string& filename) {
  cv::Mat img = imread(data_path_ + filename, cv::IMREAD_COLOR);
  cv::cvtColor(img, img, CV_BGR2GRAY);
  return img;
}

void Setup() {
  data_path_ = __FILE__;
  std::string this_filename = "superpoint_tests.cpp";
  data_path_.erase(data_path_.end() - this_filename.length(), data_path_.end());
  data_path_ += "test_data/";

  ReadMatches(data_path_ + "matches.txt", image_left_gt_matches_,
              image_right_gt_matches_);

  model_file_path_ = __FILE__;
  std::string path_from_module = "tests/superpoint_tests.cpp";
  model_file_path_.erase(model_file_path_.end() - path_from_module.length(),
                         model_file_path_.end());
  model_file_path_ += "data/models/superpoint.pt";
}

/*
TEST_CASE("Test feature extraction.") {
  Setup();

  // get images
  cv::Mat image_left = GetImage("kronan1.jpg");
  cv::Mat image_right = GetImage("kronan2.jpg");

  // extract features and descriptors
  //std::shared_ptr<beam_cv::SuperPointModel> model =
  //    std::make_shared<beam_cv::SuperPointModel>(model_file_path_);

  int grid_size = image_left.rows / 5;
  beam_cv::SuperPointDetector detector(model_file_path_, 300, 0.3, 100, 40,
grid_size, false); beam_cv::SuperPointDescriptor descriptor(model, false);

  std::vector<cv::KeyPoint> keypoints_left =
      detector.DetectFeatures(image_left);
  cv::Mat descriptors_left =
      descriptor.ExtractDescriptors(image_left, keypoints_left);

  std::vector<cv::KeyPoint> keypoints_right =
      detector.DetectFeatures(image_right);
  cv::Mat descriptors_right =
      descriptor.ExtractDescriptors(image_right, keypoints_right);

  // Get matches
  beam_cv::FLANNMatcher matcher;
  std::vector<cv::DMatch> matches = matcher.MatchDescriptors(
      descriptors_left, descriptors_right, keypoints_left, keypoints_right);

  // Draw keypoints and matches
  if (show_results_ || save_results_) {
    cv::Mat image_matches;
    cv::drawMatches(image_left, keypoints_left, image_right, keypoints_right,
                    matches, image_matches, cv::Scalar::all(-1),
                    cv::Scalar::all(-1), std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    cv::Mat image_left_w_keypoints;
    cv::drawKeypoints(image_left, keypoints_left, image_left_w_keypoints,
                      cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::Mat image_right_w_keypoints;
    cv::drawKeypoints(image_right, keypoints_right, image_right_w_keypoints,
                      cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    if (show_results_) {
      cv::imshow("Matches", image_matches);
      cv::imshow("Image Keypoints Left", image_left_w_keypoints);
      cv::imshow("Image Keypoints Right", image_right_w_keypoints);
      cv::waitKey();
    }

    if (save_results_) {
      if (!boost::filesystem::exists(save_path_)) {
        boost::filesystem::create_directory(
            boost::filesystem::path(save_path_));
      }
      std::cout << "Saving results to " << save_path_ << "\n";
      cv::imwrite(save_path_ + "image_matches.jpg", image_matches);
      cv::imwrite(save_path_ + "image_left_w_keypoints.jpg",
                  image_left_w_keypoints);
      cv::imwrite(save_path_ + "image_right_w_keypoints.jpg",
                  image_right_w_keypoints);
    }
  }

  REQUIRE(keypoints_left.size() > 200);
  REQUIRE(keypoints_right.size() > 200);
  // TODO: this currently fails - need to fix
  // REQUIRE(keypoints_left.size() < 300);
  // REQUIRE(keypoints_right.size() < 300);
  REQUIRE(matches.size() > 50);
}
*/

TEST_CASE("Test Ladybug Images without resetting model") {
  Setup();

  cv::Mat ladybug_image_1 = GetImage("ladybug_image_1.jpg");
  cv::Mat ladybug_image_2 = GetImage("ladybug_image_2.jpg");
  cv::Mat ladybug_image_3 = GetImage("ladybug_image_3.jpg");

  // extract features and descriptors
  // std::shared_ptr<beam_cv::SuperPointModel> model =
  //     std::make_shared<beam_cv::SuperPointModel>(model_file_path_);

  int grid_size = ladybug_image_3.rows / 5;
  beam_cv::SuperPointDetector detector(model_file_path_, 1000, 0.01, 0, 10, 0,
                                       false);
  beam_cv::SuperPointDescriptor descriptor(model_file_path_, 1000, 0.01, 0, 10,
                                           0, false);

  std::cout << "TEST1\n";
  std::vector<cv::KeyPoint> keypoints_1 =
      detector.DetectFeatures(ladybug_image_1);
  std::cout << "TEST2\n";
  cv::Mat descriptors_1;
  descriptors_1 = descriptor.ExtractDescriptors(ladybug_image_1, keypoints_1);
  std::cout << "TEST3\n";
  std::vector<cv::KeyPoint> keypoints_2 =
      detector.DetectFeatures(ladybug_image_2);

  cv::Mat descriptors_2;
  descriptors_2 = descriptor.ExtractDescriptors(ladybug_image_2, keypoints_2);

  std::vector<cv::KeyPoint> keypoints_3 =
      detector.DetectFeatures(ladybug_image_3);
  cv::Mat descriptors_3;
  descriptors_3 = descriptor.ExtractDescriptors(ladybug_image_3, keypoints_3);

  std::cout << "descriptors_1.size():  " << descriptors_1.size() << "\n";
  std::cout << "descriptors_2.size():  " << descriptors_2.size() << "\n";
  std::cout << "descriptors_3.size():  " << descriptors_3.size() << "\n";
  std::cout << "keypoints_1.size():  " << keypoints_1.size() << "\n";
  std::cout << "keypoints_2.size():  " << keypoints_2.size() << "\n";
  std::cout << "keypoints_3.size():  " << keypoints_3.size() << "\n";

  if (show_results_ || save_results_) {
    cv::Mat image_1_w_keypoints;
    cv::drawKeypoints(ladybug_image_1, keypoints_1, image_1_w_keypoints,
                      cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::Mat image_2_w_keypoints;
    cv::drawKeypoints(ladybug_image_2, keypoints_2, image_2_w_keypoints,
                      cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    cv::Mat image_3_w_keypoints;
    cv::drawKeypoints(ladybug_image_3, keypoints_3, image_3_w_keypoints,
                      cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    if (show_results_) {
      cv::imshow("Image 1", image_1_w_keypoints);
      cv::imshow("Image 2", image_2_w_keypoints);
      cv::imshow("Image 3", image_3_w_keypoints);
      cv::waitKey();
    }

    if (save_results_) {
      if (!boost::filesystem::exists(save_path_)) {
        boost::filesystem::create_directory(
            boost::filesystem::path(save_path_));
      }
      std::cout << "Saving results to " << save_path_ << "\n";
      cv::imwrite(save_path_ + "LadybugImage1.jpg", image_1_w_keypoints);
      cv::imwrite(save_path_ + "LadybugImage2.jpg", image_2_w_keypoints);
      cv::imwrite(save_path_ + "LadybugImage3.jpg", image_3_w_keypoints);
    }
  }

  // Get matches
  beam_cv::FLANNMatcher matcher;

  std::vector<cv::DMatch> matches12 = matcher.MatchDescriptors(
      descriptors_1, descriptors_2, keypoints_1, keypoints_2);

  std::vector<cv::DMatch> matches13 = matcher.MatchDescriptors(
      descriptors_1, descriptors_3, keypoints_1, keypoints_3);

  std::vector<cv::DMatch> matches23 = matcher.MatchDescriptors(
      descriptors_2, descriptors_3, keypoints_2, keypoints_3);

  // Draw keypoints and matches
  if (show_results_ || save_results_) {
    cv::Mat image_matches12;
    cv::drawMatches(ladybug_image_1, keypoints_1, ladybug_image_2, keypoints_2,
                    matches12, image_matches12, cv::Scalar::all(-1),
                    cv::Scalar::all(-1), std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    cv::Mat image_matches13;
    cv::drawMatches(ladybug_image_1, keypoints_1, ladybug_image_3, keypoints_3,
                    matches13, image_matches13, cv::Scalar::all(-1),
                    cv::Scalar::all(-1), std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    cv::Mat image_matches23;
    cv::drawMatches(ladybug_image_2, keypoints_2, ladybug_image_3, keypoints_3,
                    matches23, image_matches23, cv::Scalar::all(-1),
                    cv::Scalar::all(-1), std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    if (show_results_) {
      cv::imshow("Matches12", image_matches12);
      cv::imshow("Matches13", image_matches13);
      cv::imshow("Matches23", image_matches23);
      cv::waitKey();
    }

    if (save_results_) {
      cv::imwrite(save_path_ + "LadybugMatches12.jpg", image_matches12);
      cv::imwrite(save_path_ + "LadybugMatches13.jpg", image_matches13);
      cv::imwrite(save_path_ + "LadybugMatches23.jpg", image_matches23);
    }
  }

  // REQUIRE(keypoints_1.size() > 100);
  // REQUIRE(keypoints_2.size() > 100);
  // REQUIRE(keypoints_3.size() > 100);
  // REQUIRE(matches12.size() > 30);
  // REQUIRE(matches13.size() > 30);
  // REQUIRE(matches23.size() > 30);
}

// TODO: creat test for pose estimating using these detectors/features. See:
// https://github.com/BEAMRobotics/libbeam/blob/add_geometry_methods/beam_cv/tests/feature_tests.cpp
