#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include <beam_cv/descriptors/Descriptors.h>
#include <beam_cv/detectors/Detectors.h>
#include <beam_cv/matchers/Matchers.h>
#include <beam_cv/tracker/Tracker.h>

#include <beam_utils/filesystem.hpp>
#include <beam_utils/time.hpp>

std::shared_ptr<beam_cv::Matcher> matcher =
    std::make_shared<beam_cv::FLANNMatcher>(beam_cv::FLANN::KDTree, 0.8, true,
                                            true, cv::FM_RANSAC, 1);
std::shared_ptr<beam_cv::Descriptor> descriptor =
    std::make_shared<beam_cv::ORBDescriptor>();
std::shared_ptr<beam_cv::Detector> detector =
    std::make_shared<beam_cv::ORBDetector>();

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

TEST_CASE("Test adding images to tracker.") {
  std::vector<cv::Mat> images = ReadImageSequence();
  ros::Time::init();
  beam_cv::Tracker tracker(detector, descriptor, matcher, 10);
  for (int i = 0; i < 10; i++) {
    tracker.AddImage(images[i], ros::Time::now());
  }

  struct timespec t;
  beam::tic(&t);
  tracker.AddImage(images[10], ros::Time::now());
  float elapsed = beam::toc(&t);
  BEAM_INFO("Adding image to window (size 10): {} seconds", elapsed);

  std::vector<std::vector<beam_containers::LandmarkMeasurement<int>>>
      feature_tracks;
  REQUIRE_THROWS(feature_tracks = tracker.GetTracks(11));
  REQUIRE_NOTHROW(feature_tracks = tracker.GetTracks(5));

  std::vector<beam_containers::LandmarkMeasurement<int>> track =
      feature_tracks[0];
  beam_containers::LandmarkMeasurement<int> lm1 = track[0];
  beam_containers::LandmarkMeasurement<int> lm2 = track[1];
  beam_containers::LandmarkMeasurement<int> lm3 = track[2];
  std::cout << "lm id: " << lm1.landmark_id << std::endl;
  std::cout << "image #: " << lm1.image << std::endl;
  std::cout << "lm id: " << lm2.landmark_id << std::endl;
  std::cout << "image #: " << lm2.image << std::endl;
  std::cout << "lm id: " << lm3.landmark_id << std::endl;
  std::cout << "image #: " << lm3.image << std::endl;
}
