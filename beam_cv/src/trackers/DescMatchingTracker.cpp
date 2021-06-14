#include <beam_cv/trackers/DescMatchingTracker.h>

namespace beam_cv {

DescMatchingTracker::DescMatchingTracker(
    std::shared_ptr<beam_cv::Detector> detector,
    std::shared_ptr<beam_cv::Descriptor> descriptor,
    std::shared_ptr<beam_cv::Matcher> matcher, int window_size)
    : Tracker(window_size),
      detector_(detector),
      descriptor_(descriptor),
      matcher_(matcher) {}

void DescMatchingTracker::DetectAndCompute(const cv::Mat& image,
                               std::vector<cv::KeyPoint>& keypoints,
                               cv::Mat& descriptor) {
  keypoints = detector_->DetectFeatures(image);
  descriptor = descriptor_->ExtractDescriptors(image, keypoints);
}

std::map<int, uint64_t>
    DescMatchingTracker::RegisterKeypoints(const std::vector<cv::KeyPoint>& curr_kp,
                               const cv::Mat& curr_desc,
                               const std::vector<cv::DMatch>& matches) {
  // Maps current keypoint indices to IDs
  std::map<int, uint64_t> curr_ids;

  for (const auto& m : matches) {
    // Check to see if ID has already been assigned to keypoint
    if (prev_ids_.count(m.queryIdx)) {
      // If so, assign that ID to current map.
      auto id = prev_ids_.at(m.queryIdx);
      curr_ids[m.trainIdx] = id;
      // Extract value of keypoint.
      Eigen::Vector2d landmark = ConvertKeypoint(curr_kp.at(m.trainIdx));
      cv::Mat landmark_descriptor = curr_desc.row(m.trainIdx);
      auto img_count = img_times_.size() - 1;
      // Emplace LandmarkMeasurement into LandmarkMeasurementContainer
      landmarks_.Emplace(img_times_.at(img_count), sensor_id_,
                         curr_ids.at(m.trainIdx), img_count, landmark,
                         landmark_descriptor);
    } else {
      // Else, assign new ID
      auto id = GenerateFeatureID();
      prev_ids_[m.queryIdx] = id;
      curr_ids[m.trainIdx] = prev_ids_.at(m.queryIdx);
      // Since keypoint was not a match before, need to add previous and
      // current points to measurement container
      Eigen::Vector2d prev_landmark = ConvertKeypoint(prev_kp_.at(m.queryIdx));
      Eigen::Vector2d curr_landmark = ConvertKeypoint(curr_kp.at(m.trainIdx));
      cv::Mat curr_descriptor = curr_desc.row(m.trainIdx);
      cv::Mat prev_descriptor = prev_desc_.row(m.queryIdx);

      // Find previous and current times from lookup table
      // Subtract one, since images are zero indexed.
      auto curr_img = img_times_.size() - 1;
      auto prev_img = curr_img - 1;
      const auto& prev_time = img_times_.at(prev_img);
      const auto& curr_time = img_times_.at(curr_img);
      // Add previous and current landmarks to container
      landmarks_.Emplace(prev_time, sensor_id_, prev_ids_.at(m.queryIdx),
                         prev_img, prev_landmark, prev_descriptor);

      landmarks_.Emplace(curr_time, sensor_id_, curr_ids.at(m.trainIdx),
                         curr_img, curr_landmark, curr_descriptor);
    }
  }
  return curr_ids;
}

std::map<int, uint64_t> DescMatchingTracker::Match(const cv::Mat& image,
                                       std::vector<cv::KeyPoint>& kp,
                                       cv::Mat& desc,
                                       std::vector<cv::DMatch>& matches) {
  std::map<int, uint64_t> ids;
  // return empty ids if its first image
  if (img_times_.size() == 0) {
    return ids;
  } else {
    // Detect, describe, and match keypoints
    DetectAndCompute(image, kp, desc);
    matches = matcher_->MatchDescriptors(prev_desc_, desc, prev_kp_, kp);
    // fill id map
    for (const auto& m : matches) {
      if (prev_ids_.count(m.queryIdx)) {
        auto id = prev_ids_.at(m.queryIdx);
        ids[m.trainIdx] = id;
      }
    }
  }
  return ids;
}

void DescMatchingTracker::Register(const ros::Time& current_time,
                       const std::vector<cv::KeyPoint>& kp, const cv::Mat& desc,
                       const std::vector<cv::DMatch>& matches) {
  if (img_times_.size() == 0) { return; }
  // Register the time this image
  TimestampImage(current_time);
  // Register keypoints with IDs, and store Landmarks in container
  std::map<int, uint64_t> curr_ids = RegisterKeypoints(kp, desc, matches);
  // Set previous ID map to be the current one, and reset
  prev_ids_.swap(curr_ids);
  // Update previous keypoints and descriptors
  prev_kp_ = kp;
  prev_desc_ = desc;
}

void DescMatchingTracker::AddImage(const cv::Mat& image, const ros::Time& current_time) {
  // Check if this is the first image being tracked.
  if (img_times_.size() == 0) {
    // Register the time this image
    TimestampImage(current_time);
    // Detect features within first image. No tracks can be generated yet.
    DetectAndCompute(image, prev_kp_, prev_desc_);
  } else {
    // Variables for feature detection, description, and matching
    std::vector<cv::KeyPoint> curr_kp;
    cv::Mat curr_desc;
    std::vector<cv::DMatch> matches;
    // Variables for bookkeeping
    std::map<int, uint64_t> curr_ids;
    // Detect, describe, and match keypoints
    DetectAndCompute(image, curr_kp, curr_desc);
    matches =
        matcher_->MatchDescriptors(prev_desc_, curr_desc, prev_kp_, curr_kp);
    // Register the time this image
    TimestampImage(current_time);
    // Register keypoints with IDs, and store Landmarks in container
    curr_ids = RegisterKeypoints(curr_kp, curr_desc, matches);
    // Set previous ID map to be the current one, and reset
    prev_ids_.swap(curr_ids);
    // Update previous keypoints and descriptors
    prev_kp_ = curr_kp;
    prev_desc_ = curr_desc;
  }
}

} // namespace beam_cv
