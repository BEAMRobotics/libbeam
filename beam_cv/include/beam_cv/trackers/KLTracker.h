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
#include <beam_cv/trackers/Tracker.h>

namespace beam_cv {

/**
 * @brief Image tracker class that uses opencv's KLT tracker
 */
class KLTracker : public Tracker {
public:
  struct Params {
    /** value between 0 and 1 which determines when to sample new keypoints. If
     * the current number of keypoints, divided by the number of keypoints first
     * extracted is lower than this value, this will resample keypoints to bring
     * up the count to the original count.*/
    double keypoint_resample_percentage{0.7};

    /** size of the search window at each pyramid level. */
    int win_size_u{21};
    int win_size_v{21};

    /** 0-based maximal pyramid level number; if set to 0, pyramids are not used
     * (single level), if set to 1, two levels are used, and so on; if pyramids
     * are passed to input then algorithm will use as many levels as pyramids
     * have but no more than maxLevel. */
    int max_level{3};

    /** parameter, specifying the termination criteria of the iterative search
     * algorithm (after the specified maximum number of iterations
     * criteria.maxCount or when the search window moves by less than
     * criteria.epsilon. */
    cv::TermCriteria criteria = cv::TermCriteria(
        cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

    /** load params from a json config file. If empty, it will use default
     * params. */
    void LoadFromJson(const std::string& config_path);
  };

  /**
   * @brief Constructor with default variables
   * @param detector pointer to detector object (FAST, ORB, etc...)
   * @param descriptor pointer to descriptor object (BRISK, ORB, etc...). This
   * is not needed by KLT but the landmark container usually holds a descriptor
   * for each keypoint. If this is set to nullptr (or not provided in this
   * constructor) then the descriptor field for each keypont will be left empty.
   * @param window_size For online, sliding window tracker operation. Maintains
   * memory by clearing out values from the measurement container that are
   * outside of this time window. If set to zero (default), all measurements are
   * kept for offline use.
   */
  KLTracker(std::shared_ptr<beam_cv::Detector> detector,
            std::shared_ptr<beam_cv::Descriptor> descriptor = nullptr,
            int window_size = 100);

  /**
   * @brief Constructor with custom variables
   * @param params see struct defined above
   * @param detector pointer to detector object (FAST, ORB, etc...)
   * @param descriptor pointer to descriptor object (BRISK, ORB, etc...). This
   * is not needed by KLT but the landmark container usually holds a descriptor
   * for each keypoint. If this is set to nullptr (or not provided in this
   * constructor) then the descriptor field for each keypont will be left empty.
   * @param window_size For online, sliding window tracker operation. Maintains
   * memory by clearing out values from the measurement container that are
   * outside of this time window. If set to zero (default), all measurements are
   * kept for offline use.
   */
  KLTracker(const Params& params, std::shared_ptr<beam_cv::Detector> detector,
            std::shared_ptr<beam_cv::Descriptor> descriptor = nullptr,
            int window_size = 100);

  /**
   * @brief Default destructor
   */
  ~KLTracker() = default;

  /**
   * @brief Validate params and if not set correctly, output error and set to
   * default
   */
  void ValidateParams();

  /**
   * @brief Track features within an image (presumably the next in a
   * sequence).
   * @param image the image to add.
   * @param current_time the time at which the image was captured
   */
  void AddImage(const cv::Mat& image, const ros::Time& current_time) override;

  /**
   * @brief Purges the container but retains the current id value
   */
  void Reset() override;

private:
  /**
   * @brief This extracts keypoints and descriptors (if descriptor object
   * initialized) and fills in the following member variables: curr_ids_,
   * tracked_features_count_, prev_kp_, prev_desc_ (optional), prev_image_
   * @param image first image of a series
   */
  void DetectInitialFeatures(const cv::Mat& image);

  /**
   * @brief this function takes all successful keypoint that were tracked, moves
   * them to the tracked keypoints (if not already there), then adds they
   * tracked keypoints to the landmark container
   * @param status result of tracked which has a 1 if the keypoint was
   * succesfully tracked, or 0 otherwise.
   * @param image current image that keypoints come from. This is used to
   * extract descriptors
   * @param new_points_start_id if new points have been added, we want to add
   * those landmarks detections from the current image and the previous image.
   * This variable describes the index at which the new keypoints start at
   */
  void RegisterKeypoints(const std::vector<uchar>& status, const cv::Mat& image,
                         uint32_t new_points_start_id);

  /**
   * @brief  add points from the previous images that have not been added to the
   * tracker yet. This occurs if either:
   *
   * (A) the current image is the second image added (keypoints from first image
   * can't be added until they have been tracked at least one), or
   *
   * (B) we have added new keypoints to the tracker for this image because the
   * number of features has dropped below the threshold
   *
   * @param status result of tracked which has a 1 if the keypoint was
   * succesfully tracked, or 0 otherwise.
   * @param new_points_start_id if new points have been added, we want to add
   * those landmarks detections from the current image and the previous image.
   * This variable describes the index at which the new keypoints start at
   */
  void AddPrevTrackedLandmarks(const std::vector<uchar>& status,
                               uint32_t new_points_start_id);

  /**
   * @brief this function checkes the number of tracked keypoints, and if it
   * drops below the threshold in params, then it'll throw away the non tracked
   * keypoints and extract new ones
   * @param image image we want to extract new keypoints from
   * @return new_points_start_id index of the prev_kp_ vector where the new
   * points begin. This is needed so that when we add new landmarks, we can add
   * the detection from the previous and current image
   */
  uint32_t ExtractNewKeypoints(const cv::Mat& image);

  Params params_;

  // detector and descriptor
  std::shared_ptr<beam_cv::Detector> detector_;
  std::shared_ptr<beam_cv::Descriptor> descriptor_;

  // keep track of keypoint ids and which ones are tracked and untracked. first:
  // unique id, second: is_tracked
  std::vector<std::pair<uint64_t, bool>> curr_ids_;
  uint32_t tracked_features_count_{0};
  uint32_t original_feature_count_{0};

  // Need to store keypoints & image from the previous timestep
  std::vector<cv::Point2f> prev_kp_;
  cv::Mat prev_image_;
  cv::Mat prev_desc_;

  // also store current keypoints and descriptors for ease of use
  std::vector<cv::Point2f> curr_kp_;
  cv::Mat curr_desc_;
};

} // namespace beam_cv
