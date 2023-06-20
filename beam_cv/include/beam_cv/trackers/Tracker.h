/**
 * @file
 * Feature tracker implementation.
 * @ingroup beam_cv
 */
#pragma once

#include <chrono>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>

#include <beam_containers/LandmarkContainer.h>
#include <beam_containers/LandmarkMeasurement.h>
#include <beam_cv/Utils.h>
#include <beam_utils/utils.h>

namespace beam_cv {

typedef std::vector<beam_containers::LandmarkMeasurement> FeatureTrack;

/**
 * @brief Enum class for different tracker types
 */
enum class TrackerType { DESCMATCHING = 0, KL };

// Map for storing string input
static std::map<std::string, TrackerType> TrackerTypeStringMap = {
    {"DESCMATCHING", TrackerType::DESCMATCHING},
    {"KL", TrackerType::KL}};

// Map for storing int input
static std::map<uint8_t, TrackerType> TrackerTypeIntMap = {
    {0, TrackerType::DESCMATCHING},
    {1, TrackerType::KL}};

// function for listing types of Tracker available
inline std::string GetTrackerTypes() {
  std::string types;
  for (auto it = TrackerTypeStringMap.begin(); it != TrackerTypeStringMap.end();
       it++) {
    types += it->first;
    types += ", ";
  }
  types.erase(types.end() - 2, types.end());
  return types;
}

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
   * @brief Modified constructor for when outlier rejection is wanted
   * @param K camera matrix
   */
  Tracker(const Eigen::Matrix3d& K, int window_size = 0);

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
   * (defined in base class)
   *
   * @param image the image to add. This needs to be a greyscal image
   * @param current_time the time at which the image was captured
   */
  virtual void AddImage(const cv::Mat& image,
                        const ros::Time& current_time) = 0;

  /**
   * @brief Purges the container but retains the current id value
   */
  virtual void Reset() = 0;

  /**
   * @brief Get the tracks of all features in the requested image from the
   * sequence using the image number
   * @param img_num the number of the image to obtain tracks from
   * @return tracks corresponding to all detected landmarks in the image, from
   * the start of time to the given image.
   */
  std::vector<FeatureTrack> GetTracks(const size_t img_num) const;

  /**
   * @brief Get the tracks of all features in the requested image from the
   * sequence, using the timestamp of the image
   * @param stamp the timstamp of the image to obtain tracks from
   * @return tracks corresponding to all detected landmarks in the image, from
   * the start of time to the given image.
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
  std::vector<uint64_t> GetLandmarkIDsInImage(const ros::Time& now) const;

  /**
   * @brief Get all landmark ids in a given time window
   * @param start timestamp of start
   * @param end timestamp of end
   * @return the vector landmark ids
   */
  std::vector<uint64_t> GetLandmarkIDsInWindow(const ros::Time& start,
                                               const ros::Time& end) const;

  /**
   * @brief Get feature track of a given landmark
   * @param landmark_id to get track of
   * @return the vector of landmark measurements
   */
  FeatureTrack GetTrack(uint64_t landmark_id);

  /**
   * @brief Gets the descritpro for a specific landmark
   * @param stamp timestamp of measurement
   * @param landmark_id to get track of
   * @return cv::Mat
   */
  cv::Mat GetDescriptor(const ros::Time& stamp, const uint64_t& landmark_id);

  /**
   * @brief Get the size of the LandmarkContainer
   * @return size
   */
  size_t GetLandmarkContainerSize();

  /**
   * @brief Computes the average parallax between two frames
   * @return parallax
   */
  double ComputeParallax(const ros::Time& frame1, const ros::Time& frame2,
                         bool compute_median = false);

  /**
   * @brief Sets the sensor id
   */
  void SetSensorID(uint8_t sensor_id);

protected:
  /**
   * @brief Generate a new ID for each newly detected feature.
   * @return the assigned ID.
   */
  uint64_t GenerateFeatureID();

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

  // The sensor ID. TODO: Expand this for use with multiple cams.
  uint8_t sensor_id_ = 0;

  // vector of all image times
  std::set<uint64_t> img_times_;

  // Measurement container variables
  beam_containers::LandmarkContainer landmarks_;

  // store current id
  uint64_t id_{0};

  cv::Mat K_;
  bool use_outlier_rejection_ = false;
};

} // namespace beam_cv
