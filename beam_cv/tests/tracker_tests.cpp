#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include <beam_cv/descriptors/Descriptors.h>
#include <beam_cv/detectors/Detectors.h>
#include <beam_cv/matchers/Matchers.h>
#include <beam_cv/tracker/Tracker.h>

#include <beam_utils/filesystem.h>
#include <beam_utils/time.h>

std::shared_ptr<beam_cv::Matcher> matcher1 =
    std::make_shared<beam_cv::FLANNMatcher>();
std::shared_ptr<beam_cv::Matcher> matcher2 =
    std::make_shared<beam_cv::BFMatcher>();
std::shared_ptr<beam_cv::Descriptor> descriptor =
    std::make_shared<beam_cv::ORBDescriptor>();
std::shared_ptr<beam_cv::Detector> detector =
    std::make_shared<beam_cv::GFTTDetector>(500);

std::vector<cv::Mat> ReadImageSequence() {
  std::string libbeam_root = beam::LibbeamRoot();
  std::string image_seq_folder =
      libbeam_root + "beam_cv/tests/test_data/image_sequence/";

  std::vector<cv::Mat> images;
  for (int i = 1; i <= 11; i++) {
    cv::Mat im = cv::imread(image_seq_folder + std::to_string(i) + ".jpg",
                            cv::IMREAD_COLOR);
    images.push_back(im);
  }
  return images;
}

TEST_CASE("Test adding images to tracker - FLANN Matcher.") {
  std::vector<cv::Mat> images = ReadImageSequence();
  ros::Time::init();
  beam_cv::Tracker tracker(detector, descriptor, matcher1, 10);
  for (int i = 0; i < 10; i++) {
    tracker.AddImage(images[i], ros::Time::now());
  }

  struct timespec t;
  beam::tic(&t);
  tracker.AddImage(images[10], ros::Time::now());
  float elapsed = beam::toc(&t);
  BEAM_INFO("Adding image to window (FLANN): {} seconds", elapsed);

  std::vector<beam_cv::FeatureTrack> feature_tracks;
  REQUIRE_THROWS(feature_tracks = tracker.GetTracks(11));
  REQUIRE_NOTHROW(feature_tracks = tracker.GetTracks(5));
}

TEST_CASE("Test adding images to tracker - Brute Force Matcher.") {
  std::vector<cv::Mat> images = ReadImageSequence();
  ros::Time::init();
  beam_cv::Tracker tracker(detector, descriptor, matcher2, 10);
  for (int i = 0; i < 10; i++) {
    tracker.AddImage(images[i], ros::Time::now());
  }

  struct timespec t;
  beam::tic(&t);
  tracker.AddImage(images[10], ros::Time::now());
  float elapsed = beam::toc(&t);
  BEAM_INFO("Adding image to window (BRUTE FORCE): {} seconds", elapsed);

  std::vector<beam_cv::FeatureTrack> feature_tracks;
  REQUIRE_THROWS(feature_tracks = tracker.GetTracks(11));
  REQUIRE_NOTHROW(feature_tracks = tracker.GetTracks(5));
}
