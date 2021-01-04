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

cv::Mat image_left_;
cv::Mat image_right_;
std::vector<Eigen::Vector2i> image_left_gt_matches_;
std::vector<Eigen::Vector2i> image_right_gt_matches_;
std::string model_file_path_;
bool show_results_{false};
bool save_results_{true};
std::string save_path_ = "/tmp/superpoint_tests/";

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

void Setup() {
  std::string data_path = __FILE__;
  std::string this_filename = "superpoint_tests.cpp";
  data_path.erase(data_path.end() - this_filename.length(), data_path.end());
  data_path += "test_data/";

  ReadMatches(data_path + "matches.txt", image_left_gt_matches_,
              image_right_gt_matches_);

  image_left_ = imread(data_path + "kronan1.jpg", cv::IMREAD_COLOR);
  cv::cvtColor(image_left_, image_left_, CV_BGR2GRAY);

  image_right_ = imread(data_path + "kronan2.jpg", cv::IMREAD_COLOR);
  cv::cvtColor(image_right_, image_right_, CV_BGR2GRAY);

  model_file_path_ = __FILE__;
  std::string path_from_module = "tests/superpoint_tests.cpp";
  model_file_path_.erase(model_file_path_.end() - path_from_module.length(),
                         model_file_path_.end());
  model_file_path_ += "data/models/superpoint.pt";
}

TEST_CASE("Test feature extraction.") {
  Setup();

  // extract features and descriptors
  std::shared_ptr<beam_cv::SuperPointModel> model =
      std::make_shared<beam_cv::SuperPointModel>(model_file_path_);

  int grid_size = image_left_.rows / 5;
  bool use_cuda = false;
  // beam_cv::SuperPointDetector detector(model, 300, 0.3, 100, 40, grid_size,
  //                                      false);
  beam_cv::SuperPointDetector::Params params{.max_features = 1000,
                                             .conf_threshold = 0.1,
                                             .border = 10,
                                             .nms_dist_threshold = 40,
                                             .grid_size = grid_size,
                                             .use_cuda = use_cuda};
  beam_cv::SuperPointDetector detector(model, params);
  beam_cv::SuperPointDescriptor descriptor(model);

  std::vector<cv::KeyPoint> keypoints_left =
      detector.DetectFeatures(image_left_);

  cv::Mat descriptors_left =
      descriptor.ExtractDescriptors(image_left_, keypoints_left);

  std::vector<cv::KeyPoint> keypoints_right =
      detector.DetectFeatures(image_right_);

  cv::Mat descriptors_right =
      descriptor.ExtractDescriptors(image_right_, keypoints_right);

  // Get matches
  beam_cv::FLANNMatcher matcher;
  std::vector<cv::DMatch> matches = matcher.MatchDescriptors(
      descriptors_left, descriptors_right, keypoints_left, keypoints_right);

  // Draw keypoints and matches
  if (show_results_ || save_results_) {
    cv::Mat image_matches;
    cv::drawMatches(image_left_, keypoints_left, image_right_, keypoints_right,
                    matches, image_matches, cv::Scalar::all(-1),
                    cv::Scalar::all(-1), std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    cv::Mat image_left_w_keypoints;
    cv::drawKeypoints(image_left_, keypoints_left, image_left_w_keypoints,
                      cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::Mat image_right_w_keypoints;
    cv::drawKeypoints(image_right_, keypoints_right, image_right_w_keypoints,
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
  REQUIRE(matches.size() > 50);
}

// TODO: create test for pose estimating using these detectors/features. See:
// https://github.com/BEAMRobotics/libbeam/blob/add_geometry_methods/beam_cv/tests/feature_tests.cpp
