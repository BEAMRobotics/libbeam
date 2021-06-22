/**
 * @file
 * Feature tracker implementation.
 * @ingroup beam_cv
 */
#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <beam_cv/trackers/Tracker.h>
#include <beam_containers/LandmarkContainer.h>
#include <beam_containers/LandmarkMeasurement.h>
#include <beam_cv/Utils.h>
#include <beam_cv/descriptors/Descriptor.h>
#include <beam_cv/detectors/Detector.h>
#include <beam_cv/matchers/Matcher.h>
#include <beam_utils/utils.h>

namespace beam_cv {

/** 
 * @brief Image tracker class that uses descriptor matching
 * The Tracker class requires a feature detector, descriptor, and matcher
 * to track features over a sequence of images.*/
class DescMatchingTracker : public Tracker {
public:
  /**
   * @brief Constructor that requires a detector, descriptor and matcher
   * @param detector pointer to detector object (FAST, ORB, etc...)
   * @param descriptor pointer to descriptor object (BRISK, ORB, etc...)
   * @param matcher pointer to matcher object (BruteForceMatcher, FLANN)
   * @param window_size For online, sliding window tracker operation. Maintains
   * memory by clearing out values from the measurement container that are
   * outside of this time window. If set to zero (default), all measurements are
   * kept for offline use.
   */
  DescMatchingTracker(std::shared_ptr<beam_cv::Detector> detector,
                      std::shared_ptr<beam_cv::Descriptor> descriptor,
                      std::shared_ptr<beam_cv::Matcher> matcher,
                      int window_size = 0);

  /**
   * @brief Default destructor
   */
  ~DescMatchingTracker() = default;

  /**
   * @brief Track features within an image (presumably the next in a
   * sequence).
   * @param image the image to add.
   * @param current_time the time at which the image was captured
   */
  void AddImage(const cv::Mat& image, const ros::Time& current_time) override;

private:
  /**
   * @brief Match a given image against the current image in tracker
   * @param image the image to match against the currently stored one
   * @param kp resulting keypoints in the image
   * @param desc resulting sdescriptors in the image
   * @return map of <keypoint index: landmark id>
   */
  std::map<int, uint64_t> Match(const cv::Mat& image,
                                std::vector<cv::KeyPoint>& kp, cv::Mat& desc,
                                std::vector<cv::DMatch>& matches);

  /**
   * @brief Detects features and computes descriptors using the
   * detector and descriptor.
   * @param image
   * @param keypoints
   * @param descriptor
   */
  void DetectAndCompute(const cv::Mat& image,
                        std::vector<cv::KeyPoint>& keypoints,
                        cv::Mat& descriptor);

  /**
   * @brief Registers the latest matched keypoints with IDs. Assigns a new ID
   * if one has not already been provided.
   * @param curr_kp the keypoints detected in the current image.
   * @param matches the matches between the current and previous images.
   * @return the map corresponding current keypoints to IDs.
   */
  std::map<int, size_t>
      RegisterKeypoints(const std::vector<cv::KeyPoint>& curr_kp,
                        const cv::Mat& curr_desc,
                        const std::vector<cv::DMatch>& matches);

  /** objects needed for matching descriptors */
  std::shared_ptr<beam_cv::Detector> detector_;
  std::shared_ptr<beam_cv::Descriptor> descriptor_;
  std::shared_ptr<beam_cv::Matcher> matcher_;

  // map from prev keypoint index -> unique keypoint id
  std::map<int, size_t> prev_ids_; 

  // Descriptors & keypoints from the previous timestep
  cv::Mat prev_desc_;
  std::vector<cv::KeyPoint> prev_kp_;
};

} // namespace beam_cv
