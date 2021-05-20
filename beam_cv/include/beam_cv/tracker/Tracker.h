/**
 * @file
 * Feature tracker implementation.
 * @ingroup beam_cv
 */
#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <beam_containers/LandmarkContainer.h>
#include <beam_containers/LandmarkMeasurement.h>
#include <beam_cv/Utils.h>
#include <beam_cv/descriptors/Descriptor.h>
#include <beam_cv/detectors/Detector.h>
#include <beam_cv/matchers/Matcher.h>
#include <beam_utils/utils.h>

namespace beam_cv {

typedef std::vector<beam_containers::LandmarkMeasurement> FeatureTrack;

/** Image tracker class.
 * The Tracker class is templated on a feature detector, descriptor, and matcher
 * to track features over a sequence of images.*/
class Tracker {
public:
  /** @brief Default constructor
   * @param detector detector object (FAST, ORB, etc...)
   * @param descriptor descriptor object (BRISK, ORB, etc...)
   * @param matcher matcher object (BruteForceMatcher, FLANN)
   * @param window_size For online, sliding window tracker operation. Maintains
   * memory by clearing out values from the measurement container that are
   * outside of this time window. If set to zero (default), all measurements are
   * kept for offline use.
   */
  Tracker(std::shared_ptr<beam_cv::Detector> detector,
          std::shared_ptr<beam_cv::Descriptor> descriptor,
          std::shared_ptr<beam_cv::Matcher> matcher, int window_size = 0)
      : detector(detector),
        descriptor(descriptor),
        matcher(matcher),
        window_size_(window_size) {}

  /**
   * @brief Default destructor
   */
  ~Tracker() = default;

  /** @brief Get the tracks of all features in the requested image from the
   * sequence.
   * @param img_num the number of the image to obtain tracks from
   * @return tracks corresponding to all detected landmarks in the image, from
   * the start of time to the given image.
   */
  std::vector<FeatureTrack> GetTracks(const size_t img_num) const;

  /** @brief Track features within an image (presumably the next in a sequence).
   * @param image the image to add.
   * @param current_time the time at which the image was captured
   */
  void AddImage(const cv::Mat& image, const ros::Time& current_time);

  /** @brief Draw tracks for the requested image.
   * @param img_num the number of the image within the sequence
   * @param image the image to draw the tracks on
   * @return the image with the tracks illustrated as arrows.
   */
  cv::Mat DrawTracks(const std::vector<FeatureTrack>& feature_tracks,
                     const cv::Mat& image) const;

  /** @brief Offline feature tracking, using list of images already loaded.
   * @param image_sequence the sequence of images to analyze.
   * @return the vector of FeatureTracks in each image.
   */
  std::vector<std::vector<FeatureTrack>>
      OfflineTracker(const std::vector<cv::Mat>& image_sequence);

  /** @brief Get value of a landmark at time t
   * @param t time to look for
   * @param landmark_id to retrieve
   * @return the pixel of the landmark at time t
   */
  Eigen::Vector2d Get(const ros::Time& t, uint64_t landmark_id) const;

  /** @brief Get all landmark ids in a given image at time t
   * @param now timestamp of image
   * @return the vector landmark ids
   */
  std::vector<uint64_t> GetLandmarkIDsInImage(
      const ros::Time& now,
      ros::Duration threshold = ros::Duration(0.000001)) const;

  /** @brief Get feature track of a given landmark
   * @param landmark_id to get track of
   * @return the vector of landmark measurements
   */
  FeatureTrack GetTrack(uint64_t landmark_id);

  std::shared_ptr<beam_cv::Detector> detector;
  std::shared_ptr<beam_cv::Descriptor> descriptor;
  std::shared_ptr<beam_cv::Matcher> matcher;

  /** The size of the LandmarkContainer */
  size_t lmc_size = 0;

private:
  /** For online, sliding window tracker operation. Maintains memory by
   *  clearing out values from the measurement container that are outside of
   *  this time window.
   *  If set to zero (default), all measurements are kept for offline use.
   */
  size_t window_size_;

  /** If in sliding window mode, this represents the highest image number that
   *  can be requested to extract tracks from.
   */
  size_t cleared_img_threshold_ = 0;

  // Keypoints and descriptors from the previous timestep
  std::vector<cv::KeyPoint> prev_kp_;
  cv::Mat prev_desc_;

  // Correspondence maps
  std::map<int, size_t> prev_ids_;
  std::map<size_t, ros::Time> img_times_;

  // Measurement container variables
  beam_containers::LandmarkContainer<beam_containers::LandmarkMeasurement>
      landmarks_;

  // The sensor ID. TODO: Expand this for use with multiple cams.
  int sensor_id_ = 0;

  /** @brief Generate a new ID for each newly detected feature.
   * @return the assigned ID.
   */
  uint64_t GenerateFeatureID() const {
    static uint64_t id = 0;
    return id++;
  }

  /** @brief Detects features and computes descriptors using the
   * detector and descriptor.
   * @param image
   * @param keypoints
   * @param descriptor
   */
  void DetectAndCompute(const cv::Mat& image,
                        std::vector<cv::KeyPoint>& keypoints,
                        cv::Mat& descriptor);

  /** @brief Register the current time with the current img_count
   * @param current_time the time at which this image was received
   */
  void TimestampImage(const ros::Time& current_time);

  /** @brief Cleans out the LandmarkMeasurementContainer for images outside the
   *  requested window_size.
   *  @param img the image to remove landmark information for.
   */
  void PurgeContainer(const int img);

  /** @brief Registers the latest matched keypoints with IDs. Assigns a new ID
   * if one has not already been provided.
   * @param curr_kp the keypoints detected in the current image.
   * @param matches the matches between the current and previous images.
   * @return the map corresponding current keypoints to IDs.
   */
  std::map<int, size_t>
      RegisterKeypoints(const std::vector<cv::KeyPoint>& curr_kp,
                        const cv::Mat& curr_desc,
                        const std::vector<cv::DMatch>& matches);
}; // namespace beam_cv

} // namespace beam_cv

#include "impl/Tracker.hpp"
