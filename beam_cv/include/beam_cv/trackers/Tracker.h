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
#include <beam_utils/utils.h>

namespace beam_cv {

typedef std::vector<beam_containers::LandmarkMeasurement> FeatureTrack;

/**
 * @brief Image tracker class base class. This class defines the interface and
 * implements some common functionality for all tracker classes
 */
class Tracker {
public:
  /**
   * @brief constructor
   * @param window_size For online, sliding window tracker operation. Maintains
   * memory by clearing out values from the measurement container that are
   * outside of this time window. If set to zero (default), all measurements are
   * kept for offline use.
   */
  Tracker(int window_size = 0);

  /**
   * @brief Default destructor
   */
  ~Tracker() = default;

  /**
   * @brief Pure virtual method for tracking and adding tracked features in a
   * new image to the tracker. We assume that this function is called on images
   * in sequence. 
   * 
   * IMPORTANT NOTE: this function must call PurgeContainer
   * (defined in base class) after adding the new tracked features to the
   * tracker.
   * 
   * @param image the image to add.
   * @param current_time the time at which the image was captured
   */
  virtual void AddImage(const cv::Mat& image,
                        const ros::Time& current_time) = 0;

  /**
   * @brief Get the tracks of all features in the requested image from the
   * sequence.
   * @param img_num the number of the image to obtain tracks from
   * @return tracks corresponding to all detected landmarks in the image, from
   * the start of time to the given image.
   */
  std::vector<FeatureTrack> GetTracks(const size_t img_num) const;

  /**
   * @brief todo: implement
   */
  std::vector<FeatureTrack> GetTracks(const ros::Time& stamp) const;

  /**
   * @brief Draw tracks for the requested image.
   * @param img_num the number of the image within the sequence
   * @param image the image to draw the tracks on
   * @return the image with the tracks illustrated as arrows.
   */
  cv::Mat DrawTracks(const std::vector<FeatureTrack>& feature_tracks,
                     const cv::Mat& image) const;

  /**
   * @brief Offline feature tracking, using list of images already loaded.
   * @param image_sequence the sequence of images to analyze.
   * @return the vector of FeatureTracks in each image.
   */
  std::vector<std::vector<FeatureTrack>>
      OfflineTracker(const std::vector<cv::Mat>& image_sequence);

  /**
   * @brief Get value of a landmark at time t
   * @param t time to look for
   * @param landmark_id to retrieve
   * @return the pixel of the landmark at time t
   */
  Eigen::Vector2d Get(const ros::Time& t, uint64_t landmark_id) const;

  /**
   * @brief Get all landmark ids in a given image at time t
   * @param now timestamp of image
   * @return the vector landmark ids
   */
  std::vector<uint64_t> GetLandmarkIDsInImage(
      const ros::Time& now,
      ros::Duration threshold = ros::Duration(0.000001)) const;

  /**
   * @brief Get all landmark ids in a given time window
   * @param start timestamp of start
   * @param end timestamp of end
   * @return the vector landmark ids
   */
  std::vector<uint64_t> GetLandmarkIDsInWindow(const ros::Time& start,
                                               const ros::Time& end) const;

  /**
   *  @brief Get feature track of a given landmark
   * @param landmark_id to get track of
   * @return the vector of landmark measurements
   */
  FeatureTrack GetTrack(uint64_t landmark_id);

  /**
   * @brief Get the size of the LandmarkContainer
   * @return size
   */
  size_t GetLandmarkContainerSize();

protected:
  /**
   * @brief Generate a new ID for each newly detected feature.
   * @return the assigned ID.
   */
  uint64_t GenerateFeatureID() const;

  /**
   * @brief Register the current time with the current img_count
   * @param current_time the time at which this image was received
   */
  void TimestampImage(const ros::Time& current_time);

  /**
   * @brief Cleans out the LandmarkMeasurementContainer for images outside the
   *  requested window_size.
   */
  void PurgeContainer();

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

  // Correspondence maps
  std::map<int, size_t> prev_ids_;
  std::map<size_t, ros::Time> img_times_;

  // Measurement container variables
  beam_containers::LandmarkContainer<beam_containers::LandmarkMeasurement>
      landmarks_;

  // The sensor ID. TODO: Expand this for use with multiple cams.
  uint8_t sensor_id_ = 0;

};

} // namespace beam_cv
