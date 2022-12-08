#include <beam_cv/trackers/Tracker.h>

namespace beam_cv {

Tracker::Tracker(int window_size) : window_size_(window_size) {}

void Tracker::TimestampImage(const ros::Time& current_time) {
  img_times_.insert(current_time.toNSec());
}

void Tracker::PurgeContainer() {
  // If in online mode - sliding window
  if (window_size_ == 0 || img_times_.size() <= window_size_) { return; }

  // Need to remove all info at this particular image. Due to zero
  // indexing, subtract one for the requested image.

  // Get the time for this image.
  auto iter = img_times_.rbegin();
  std::advance(iter, window_size_);
  ros::Time time;
  time.fromNSec(*iter);

  // remove all landmarks at this time
  landmarks_.RemoveMeasurementsAtTime(time);
}

std::vector<FeatureTrack> Tracker::GetTracks(const uint64_t img_num) const {
  std::vector<FeatureTrack> feature_tracks;
  if (img_times_.size() == 0) {
    BEAM_WARN("No images added to tracker, cannot get tracks.");
    return feature_tracks;
  }
  if (img_times_.size() == 1) {
    BEAM_WARN("Only one image added to the tracker, no tracks can be "
              "generated. Returning empty tracks.");
    return feature_tracks;
  }
  if (img_num > img_times_.size() - 1) {
    BEAM_ERROR("Image requested is in the future. Returning empty tracks.");
    return feature_tracks;
  }

  auto iter = img_times_.begin();
  std::advance(iter, img_num);
  ros::Time time;
  time.fromNSec(*iter);
  return GetTracks(time);
}

std::vector<FeatureTrack> Tracker::GetTracks(const ros::Time& stamp) const {
  std::vector<FeatureTrack> feature_tracks;
  // Determine how many images have been added
  auto stamp_nsec = stamp.toNSec();

  if (stamp_nsec < *img_times_.begin()) {
    BEAM_ERROR("Requesting tracks for image time prior to the first image in "
               "the tracker. Returning empty tracks.");
    return feature_tracks;
  } else if (stamp_nsec > *img_times_.rbegin()) {
    BEAM_ERROR("Requesting tracks for image time after the last image in the "
               "tracker. Returning empty tracks.");
    return feature_tracks;
  }
  if (img_times_.find(stamp_nsec) == img_times_.end()) {
    BEAM_ERROR("Image stamp requested is not stored in the tracker. Returning "
               "empty tracks.");
    return feature_tracks;
  }

  // Extract all of the IDs visible at this time
  auto landmark_ids = landmarks_.GetLandmarkIDsInImage(stamp);

  // For each ID, get the track.
  for (const auto& l : landmark_ids) {
    // Looking for track from first image.
    ros::Time start_time;
    start_time.fromNSec(*img_times_.begin());
    FeatureTrack tracks = landmarks_.GetTrackInWindow(l, start_time, stamp);

    // Emplace new feature track back into vector
    feature_tracks.emplace_back(tracks);
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
  return landmarks_.GetValue(t, landmark_id);
}

std::vector<uint64_t>
    Tracker::GetLandmarkIDsInImage(const ros::Time& now) const {
  return landmarks_.GetLandmarkIDsInImage(now);
}

std::vector<uint64_t>
    Tracker::GetLandmarkIDsInWindow(const ros::Time& start,
                                    const ros::Time& end) const {
  return landmarks_.GetLandmarkIDsInWindow(start, end);
}

FeatureTrack Tracker::GetTrack(uint64_t landmark_id) {
  ros::Time start_time;
  start_time.fromNSec(*img_times_.begin());
  ros::Time end_time;
  auto iter_last = img_times_.end();
  iter_last--;
  end_time.fromNSec(*iter_last);
  return landmarks_.GetTrackInWindow(landmark_id, start_time, end_time);
}

cv::Mat Tracker::GetDescriptor(const ros::Time& stamp,
                               const uint64_t& landmark_id) {
  beam_containers::LandmarkMeasurement m =
      landmarks_.GetMeasurement(stamp, landmark_id);
  return m.descriptor;
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

uint64_t Tracker::GenerateFeatureID() {
  return id_++;
}

size_t Tracker::GetLandmarkContainerSize() {
  return landmarks_.size();
}

double Tracker::ComputeParallax(const ros::Time& frame1,
                                const ros::Time& frame2, bool compute_median) {
  std::vector<uint64_t> frame1_ids = GetLandmarkIDsInImage(frame1);
  std::vector<uint64_t> frame2_ids = GetLandmarkIDsInImage(frame2);
  double total_parallax = 0.0;
  double num_correspondences = 0.0;
  std::vector<double> parallaxes;
  for (auto& id : frame1_ids) {
    try {
      Eigen::Vector2d p1 = Get(frame1, id);
      Eigen::Vector2d p2 = Get(frame2, id);
      double d = beam::distance(p1, p2);
      if (compute_median) {
        parallaxes.push_back(d);
      } else {
        total_parallax += d;
        num_correspondences += 1.0;
      }
    } catch (const std::out_of_range& oor) {}
  }
  if (compute_median) {
    std::sort(parallaxes.begin(), parallaxes.end());
    return parallaxes[parallaxes.size() / 2];
  } else {
    return total_parallax / num_correspondences;
  }
}

void Tracker::SetSensorID(uint8_t sensor_id) {
  sensor_id_ = sensor_id;
}

} // namespace beam_cv
