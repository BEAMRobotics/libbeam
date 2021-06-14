#include <beam_cv/trackers/Tracker.h>

namespace beam_cv {

Tracker::Tracker(int window_size) : window_size_(window_size) {}

void Tracker::TimestampImage(const ros::Time& current_time) {
  auto img_count = img_times_.size();
  img_times_[img_count] = current_time;
}

void Tracker::PurgeContainer() {
  // If in online mode - sliding window
  if (window_size_ == 0 || img_times_.size() <= window_size_) { return; }

  size_t cleared_img_threshold = img_times_.size() - window_size_;

  // Need to remove all info at this particular image. Due to zero
  // indexing, subtract one for the requested image.

  // Get the time for this image.
  auto time = img_times_.at(cleared_img_threshold);
  // Get all IDs at this time
  auto landmarks = landmarks_.GetLandmarkIDsInWindow(time, time);
  // Delete all landmarks in the container at this time.
  for (const auto& l : landmarks) { landmarks_.Erase(time, sensor_id_, l); }
}

std::vector<FeatureTrack> Tracker::GetTracks(const uint64_t img_num) const {
  std::vector<FeatureTrack> feature_tracks;
  // Determine how many images have been added
  size_t img_count = img_times_.size() - 1;
  if (img_num > img_count) {
    throw std::out_of_range("Image requested is in the future!");
  } else if (window_size_ > 0 && img_num < cleared_img_threshold_) {
    // for non-zero window_size_, the measurement container is periodically
    // cleaned out. Therefore can only access images still with info.
    throw std::out_of_range("Image requested is outside of maintained window!");
  } else if (img_num > 0) {
    // Find the time for this image
    ros::Time img_time = img_times_.at(img_num);
    // Extract all of the IDs visible at this time
    auto landmark_ids = landmarks_.GetLandmarkIDsInWindow(img_time, img_time);
    // For each ID, get the track.
    for (const auto& l : landmark_ids) {
      // Looking for track from first image.
      ros::Time start_time = (img_times_.begin())->second;
      FeatureTrack tracks =
          landmarks_.GetTrackInWindow(sensor_id_, l, start_time, img_time);
      // Emplace new feature track back into vector
      feature_tracks.emplace_back(tracks);
    }
  }
  return feature_tracks;
}

std::vector<std::vector<FeatureTrack>>
    Tracker::OfflineTracker(const std::vector<cv::Mat>& image_sequence) {
  // FeatureTracks from current image, and all FeatureTracks
  std::vector<FeatureTrack> curr_track;
  std::vector<std::vector<FeatureTrack>> feature_tracks;
  size_t num_images = 0;
  if (!image_sequence.empty()) {
    for (auto img_it = image_sequence.begin(); img_it != image_sequence.end();
         ++img_it) {
      // Add image to tracker
      AddImage(*img_it, ros::Time::now());
      // Get tracks from this image (first should return a null track)
      curr_track = GetTracks(num_images);
      // Add current image tracks to the list of feature tracks
      feature_tracks.push_back(curr_track);
      ++num_images;
    }
  } else {
    throw std::invalid_argument("No images loaded for image stream!");
  }
  return feature_tracks;
}

Eigen::Vector2d Tracker::Get(const ros::Time& t, uint64_t landmark_id) const {
  return landmarks_.Get(t, sensor_id_, landmark_id);
}

std::vector<uint64_t>
    Tracker::GetLandmarkIDsInImage(const ros::Time& now,
                                   ros::Duration threshold) const {
  return landmarks_.GetLandmarkIDsInWindow(now - threshold, now + threshold);
}

std::vector<uint64_t>
    Tracker::GetLandmarkIDsInWindow(const ros::Time& start,
                                    const ros::Time& end) const {
  return landmarks_.GetLandmarkIDsInWindow(start, end);
}

FeatureTrack Tracker::GetTrack(uint64_t landmark_id) {
  ros::Time start_time = (img_times_.begin())->second;
  auto img_count = img_times_.size();
  ros::Time end_time = img_times_[img_count - 1];
  return landmarks_.GetTrackInWindow(sensor_id_, landmark_id, start_time,
                                     end_time);
}

cv::Mat Tracker::DrawTracks(const std::vector<FeatureTrack>& feature_tracks,
                            const cv::Mat& image) const {
  cv::Mat out_img = image;
  // Define colour for arrows
  cv::Scalar colour(0, 255, 255); // yellow
  // Draw all feature tracks on out_img
  for (const auto& ft : feature_tracks) {
    for (size_t i = 1; i < ft.size(); i++) {
      // Convert landmark values to cv::Point2f
      cv::Point2f prev = ConvertKeypoint(ft[i - 1].value);
      cv::Point2f curr = ConvertKeypoint(ft[i].value);
      // Draw arrowed line until end of feature track is reached
      cv::arrowedLine(out_img, prev, curr, colour, 2);
    }
  }
  return out_img;
}

uint64_t Tracker::GenerateFeatureID() const {
  static uint64_t id = 0;
  return id++;
}

size_t Tracker::GetLandmarkContainerSize() {
  return landmarks_.size();
}

} // namespace beam_cv
