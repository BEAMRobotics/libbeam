#include <beam_cv/trackers/KLTracker.h>

#include <time.h>

#include <beam_utils/log.h>
#include <beam_utils/math.h>
#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/video/tracking.hpp>

namespace beam_cv {

void KLTracker::Params::LoadFromJson(const std::string& config_path) {
  if (config_path.empty()) { return; }

  if (!boost::filesystem::exists(config_path)) {
    BEAM_ERROR("Invalid file path for KLTracker params, using default params. "
               "Input: {}",
               config_path);
    return;
  }

  nlohmann::json J;
  std::ifstream file(config_path);
  file >> J;
  keypoint_resample_percentage = J["keypoint_resample_percentage"];
  win_size_u = J["win_size_u"];
  win_size_v = J["win_size_v"];
  max_level = J["max_level"];
  int criteria_max_count = J["criteria_max_count"];
  double criteria_epsilon = J["criteria_epsilon"];
  criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                              criteria_max_count, criteria_epsilon);
}

KLTracker::KLTracker(std::shared_ptr<beam_cv::Detector> detector,
                     std::shared_ptr<beam_cv::Descriptor> descriptor,
                     int window_size)
    : Tracker(window_size), detector_(detector), descriptor_(descriptor) {
  ValidateParams();
}

KLTracker::KLTracker(const Params& params,
                     std::shared_ptr<beam_cv::Detector> detector,
                     std::shared_ptr<beam_cv::Descriptor> descriptor,
                     int window_size)
    : Tracker(window_size),
      params_(params),
      detector_(detector),
      descriptor_(descriptor) {
  ValidateParams();
}

void KLTracker::ValidateParams() {
  if (params_.keypoint_resample_percentage > 1 ||
      params_.keypoint_resample_percentage < 0) {
    Params params_tmp;
    BEAM_ERROR("Invalid keypoint_resample_percentage. Value must be between 0 "
               "and 1. Setting to default: {}",
               params_tmp.keypoint_resample_percentage);
    params_.keypoint_resample_percentage =
        params_tmp.keypoint_resample_percentage;
  }
}

void KLTracker::Reset() {
  // clear landmark container
  landmarks_.clear();
  img_times_.clear();

  // reset variables
  curr_ids_.clear();
  tracked_features_count_ = 0;
  original_feature_count_ = 0;
  prev_kp_.clear();
  curr_kp_.clear();
}

void KLTracker::AddImage(const cv::Mat& image, const ros::Time& current_time) {
  // Register the time this image
  TimestampImage(current_time);

  // Check if this is the first image being tracked.
  if (img_times_.size() == 1) {
    // Detect features within first image. No tracks can be generated yet.
    DetectInitialFeatures(image);
    return;
  }

  // Track keypoints
  std::vector<uchar> status;
  std::vector<float> err;
  uint32_t new_points_start_id = ExtractNewKeypoints(image);
  cv::calcOpticalFlowPyrLK(prev_image_, image, prev_kp_, curr_kp_, status, err,
                           cv::Size(params_.win_size_u, params_.win_size_v),
                           params_.max_level, params_.criteria);

  // assign current keypoints and descriptors & add to landmarks
  RegisterKeypoints(status, image, new_points_start_id);

  // Update previous keypoints & image
  prev_image_ = image.clone();
  prev_kp_ = curr_kp_;
  prev_desc_ = curr_desc_.clone();
}

void KLTracker::DetectInitialFeatures(const cv::Mat& image) {
  // get keypoints
  std::vector<cv::KeyPoint> keypoints_tmp = detector_->DetectFeatures(image);
  original_feature_count_ = keypoints_tmp.size();

  // get descriptors: this must be called before keypoints_tmp is converted and
  // stored since the descriptor may eliminate keypoints
  if (descriptor_ != nullptr) {
    prev_desc_ = descriptor_->ExtractDescriptors(image, keypoints_tmp);
    curr_desc_ = prev_desc_.clone(); // needed for sizing later on
  }

  // now we can convert keypoints to proper format that KLT needs
  cv::KeyPoint::convert(keypoints_tmp, prev_kp_);

  // generate IDs
  for (uint32_t i = 0; i < prev_kp_.size(); i++) {
    uint64_t id = GenerateFeatureID();
    curr_ids_.push_back(std::make_pair(id, true));
  }
  tracked_features_count_ = prev_kp_.size();
  prev_image_ = image.clone();
}

void KLTracker::RegisterKeypoints(const std::vector<uchar>& status,
                                  const cv::Mat& image,
                                  uint32_t new_points_start_id) {
  // if this is the second image, then all previous points are new so we need to
  // add all those landmarks, if they were tracked between these images
  if (img_times_.size() == 2) { new_points_start_id = 0; }

  // add points from the previous images that have not been added to the tracker
  // yet. This occurs if either (A) this is the second image (keypoints from
  // first image can't be added until they have been tracked at least one), or
  // (B) we have added new keypoints to the tracker
  AddPrevTrackedLandmarks(status, new_points_start_id);

  // get ros time for the last image
  ros::Time curr_time;
  curr_time.fromNSec(*img_times_.rbegin());

  // if descriptor object provided, extract descriptors from current image
  if (descriptor_ != nullptr) {
    std::vector<cv::KeyPoint> curr_kp_tmp;
    cv::KeyPoint::convert(curr_kp_, curr_kp_tmp);
    cv::Mat descriptors_tmp =
        descriptor_->ExtractDescriptors(image, curr_kp_tmp);

    if (curr_kp_tmp.size() != curr_kp_.size()) {
      int reduced_kp_counter{0};
      uint32_t total_keypoints = curr_kp_.size();
      int desc_mat_type = prev_desc_.type();
      curr_desc_ = cv::Mat(total_keypoints, prev_desc_.cols, desc_mat_type);
      for (uint32_t kp_iter = 0; kp_iter < total_keypoints; kp_iter++) {
        if (reduced_kp_counter >= descriptors_tmp.rows) { break; }
        Eigen::Vector2f diff;
        diff[0] = curr_kp_tmp[reduced_kp_counter].pt.x - curr_kp_[kp_iter].x;
        diff[1] = curr_kp_tmp[reduced_kp_counter].pt.y - curr_kp_[kp_iter].y;
        if (diff.norm() > 0.001) {
          curr_ids_.at(kp_iter).second = false;
          tracked_features_count_--;
        } else {
          curr_desc_.row(kp_iter) = descriptors_tmp.row(reduced_kp_counter);
          reduced_kp_counter++;
        }
      }
    } else {
      curr_desc_ = descriptors_tmp.clone();
    }
  }

  for (uint16_t i = 0; i < status.size() - 1; i++) {
    // if the current feature has lost tracking, skip
    if (!curr_ids_.at(i).second) { continue; }

    // if lost tracking this time, update status and continue to next keypoint
    if (!status.at(i)) {
      curr_ids_.at(i).second = false;
      tracked_features_count_--;
      continue;
    }

    // else, the keypoint is still being tracked so add landmark measurement
    Eigen::Vector2d landmark(static_cast<double>(curr_kp_.at(i).x),
                             static_cast<double>(curr_kp_.at(i).y));
    cv::Mat landmark_descriptor;
    if (descriptor_ != nullptr) { landmark_descriptor = curr_desc_.row(i); }
    landmarks_.Emplace(curr_time, sensor_id_, curr_ids_.at(i).first,
                       img_times_.size() - 1, landmark, landmark_descriptor);
  }
}

void KLTracker::AddPrevTrackedLandmarks(const std::vector<uchar>& status,
                                        uint32_t new_points_start_i) {
  auto iter = img_times_.rbegin();
  ++iter;
  ros::Time prev_time;
  prev_time.fromNSec(*iter);

  for (uint16_t i = new_points_start_i; i < status.size(); i++) {
    // if the current feature has lost tracking, skip
    if (!curr_ids_.at(i).second) { continue; }

    // if lost tracking this time, update status and continue to next keypoint
    if (!status.at(i)) {
      curr_ids_.at(i).second = false;
      tracked_features_count_--;
      continue;
    }

    const cv::Point2f& p = prev_kp_.at(i);
    Eigen::Vector2d prev_landmark(static_cast<double>(p.x),
                                  static_cast<double>(p.y));

    cv::Mat prev_descriptor;
    if (descriptor_ != nullptr) { prev_descriptor = prev_desc_.row(i); }

    // Add previous and current landmarks to container
    landmarks_.Emplace(prev_time, sensor_id_, curr_ids_.at(i).first,
                       img_times_.size() - 2, prev_landmark, prev_descriptor);
  }
}

uint32_t KLTracker::ExtractNewKeypoints(const cv::Mat& image) {
  if (tracked_features_count_ / curr_ids_.size() >=
      params_.keypoint_resample_percentage) {
    return prev_kp_.size();
  }

  // first, extract more keypoints from current image
  std::vector<cv::KeyPoint> new_kp_tmp = detector_->DetectFeatures(image);
  cv::Mat new_descriptors;
  cv::Mat combined_descriptors;
  if (descriptor_ != nullptr) {
    new_descriptors = descriptor_->ExtractDescriptors(image, new_kp_tmp);
  }
  std::vector<cv::Point2f> new_kp;
  cv::KeyPoint::convert(new_kp_tmp, new_kp);

  // next remove all non-tracked keypoints
  std::vector<std::pair<uint64_t, bool>> combined_ids;
  std::vector<cv::Point2f> combined_kp;
  for (uint32_t i = 0; i < curr_ids_.size(); i++) {
    if (curr_ids_.at(i).second) {
      combined_ids.push_back(curr_ids_[i]);
      combined_kp.push_back(prev_kp_.at(i));
      if (descriptor_ != nullptr) {
        combined_descriptors.push_back(prev_desc_.row(i));
      }
    }
  }

  uint32_t new_points_start_id = combined_ids.size();

  // finally, combine old tracked keypoints with randomly selected new
  // keypoints
  std::set<int> new_keypoints_selected;
  srand(time(NULL));
  uint32_t counter{0};

  while (combined_ids.size() < original_feature_count_) {
    counter++;

    // sample random keypoints
    uint32_t kp_selected = beam::randi(new_kp.size() - 1, 0);

    if (new_keypoints_selected.find(kp_selected) ==
        new_keypoints_selected.end()) {
      new_keypoints_selected.insert(kp_selected);
      auto id = GenerateFeatureID();
      combined_ids.push_back(std::make_pair(id, true));
      combined_kp.push_back(new_kp.at(kp_selected));
      if (descriptor_ != nullptr) {
        combined_descriptors.push_back(new_descriptors.row(kp_selected));
      }
    } else if (counter >= new_kp.size()) {
      break;
    }
  }

  // assign member variables
  curr_ids_ = combined_ids;
  prev_kp_ = combined_kp;
  prev_desc_ = combined_descriptors.clone();
  tracked_features_count_ = curr_ids_.size();

  return new_points_start_id;
}

} // namespace beam_cv
