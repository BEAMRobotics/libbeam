#include <beam_containers/LandmarkContainer.h>

#include <beam_utils/time.h>

namespace beam_containers {

bool LandmarkContainer::empty() const {
  return composite().empty();
}

size_t LandmarkContainer::size() const {
  return composite().size();
}

void LandmarkContainer::clear() {
  measurement_times_.clear();
  return composite().clear();
}

bool LandmarkContainer::Insert(const MeasurementType& m) {
  measurement_times_.insert(m.time_point.toNSec());
  auto [it, flag] = composite().insert(m);
  return flag;
}

bool LandmarkContainer::Erase(const TimeType& t, const LandmarkIdType id) {
  auto& composite = this->composite();
  auto it = composite.find(std::make_tuple(t, id));
  if (it == composite.end()) { return false; }
  composite.erase(it);
  return true;
}

ValueType LandmarkContainer::GetValue(const TimeType& t,
                                      LandmarkIdType id) const {
  const auto& composite = this->composite();
  auto iter = composite.find(std::make_tuple(t, id));
  if (iter == composite.end()) {
    // Requested key is not in this container
    throw std::out_of_range("LandmarkContainer::get");
  }
  return iter->value;
}

MeasurementType LandmarkContainer::GetMeasurement(const TimeType& t,
                                                  LandmarkIdType id) const {
  const auto& composite = this->composite();
  auto iter = composite.find(std::make_tuple(t, id));
  if (iter == composite.end()) {
    // Requested key is not in this container
    throw std::out_of_range("LandmarkContainer::get");
  }
  return *iter;
}

std::pair<landmark_container_iterator, landmark_container_iterator>
    LandmarkContainer::GetTimeWindow(const TimeType& start,
                                     const TimeType& end) const {
  // Consider a "backward" window empty
  if (start > end) { return {this->end(), this->end()}; }
  // The composite index is already sorted by time first, thus it's enough to
  // do a partial search. Find the start and end of the range.
  const auto& composite = this->composite();
  auto iter_begin = composite.lower_bound(std::make_tuple(start));
  auto iter_end = composite.upper_bound(std::make_tuple(end));
  return {iter_begin, iter_end};
}

std::vector<LandmarkIdType> LandmarkContainer::GetLandmarkIDs() const {
  return this->GetLandmarkIDsInWindow(MeasurementType::MinTime(),
                                      MeasurementType::MaxTime());
}

std::vector<LandmarkIdType>
    LandmarkContainer::GetLandmarkIDsInWindow(const TimeType& start,
                                              const TimeType& end) const {
  // Use the index sorted by landmark id
  const auto& landmark_index =
      this->storage.template get<typename landmark_container::landmark_index>();
  auto unique_ids = std::vector<LandmarkIdType>{};
  // Iterate over all measurements sorted by time first, then landmark_id.
  // Copy landmark ids into a vector, skipping consecutive equal elements and
  // elements outside the desired time window.
  for (auto& meas : landmark_index) {
    if (unique_ids.empty() || meas.landmark_id != unique_ids.back()) {
      if (meas.time_point >= start && meas.time_point <= end) {
        unique_ids.push_back(meas.landmark_id);
      }
    }
  }
  return unique_ids;
}

std::vector<LandmarkIdType>
    LandmarkContainer::GetLandmarkIDsInImage(const TimeType& img_time) const {
  // Use the index sorted by landmark id
  const auto& landmark_index =
      this->storage.template get<typename landmark_container::landmark_index>();
  auto unique_ids = std::vector<LandmarkIdType>{};
  // Iterate over all measurements sorted by time first, then landmark_id.
  // Copy landmark ids into a vector, skipping consecutive equal elements and
  // elements outside the desired time window.
  for (auto& meas : landmark_index) {
    if (unique_ids.empty() || meas.landmark_id != unique_ids.back()) {
      if (meas.time_point == img_time) {
        unique_ids.push_back(meas.landmark_id);
      }
    }
  }
  return unique_ids;
}

Track LandmarkContainer::GetTrack(const LandmarkIdType& id) const {
  return this->GetTrackInWindow(id, MeasurementType::MinTime(),
                                MeasurementType::MaxTime());
};

Track LandmarkContainer::GetTrackInWindow(const LandmarkIdType& id,
                                          const TimeType& start,
                                          const TimeType& end) const {
  // Consider a "backwards" window empty
  if (start > end) { return Track{}; }
  const auto& landmark_index =
      this->storage.template get<typename landmark_container::landmark_index>();
  // Get all measurements with desired landmark id
  const auto res = landmark_index.equal_range(id);
  // We need to get a track sorted by time, but landmark_index is not.
  // Construct a view, indexed by time, with only those measurements
  // This method is based on one described here:
  // http://www.boost.org/doc/libs/1_63_0/libs/multi_index/doc/examples.html#example6
  // While iterating, pick the measurements with desired sensor_id
  auto time_view = typename landmark_container::time_view{};
  for (auto it = res.first; it != res.second; ++it) {
    // insert a pointer to the measurement
    time_view.insert(&*it);
  }
  // Narrow down to the desired time window
  const auto iter_begin = time_view.lower_bound(start);
  const auto iter_end = time_view.upper_bound(end);
  // Build a vector holding copies of the measurements
  // Remember time_view holds pointers, so we can't copy it directly
  auto track = Track{};
  for (auto it = iter_begin; it != iter_end; ++it) { track.push_back(**it); }
  return track;
};

void LandmarkContainer::RemoveMeasurementsAtTime(const TimeType& time) {
  // Get all IDs at this time
  auto landmarks = GetLandmarkIDsInImage(time);
  // Delete all landmarks in the container at this time.
  for (const auto& l : landmarks) { Erase(time, l); }
  // erase time from time set
  measurement_times_.erase(time.toNSec());
}

double LandmarkContainer::ComputeParallax(const TimeType& t1,
                                          const TimeType& t2,
                                          bool compute_median) {
  std::vector<uint64_t> frame1_ids = GetLandmarkIDsInImage(t1);
  double total_parallax = 0.0;
  double num_correspondences = 0.0;
  std::vector<double> parallaxes;
  for (auto& id : frame1_ids) {
    try {
      Eigen::Vector2d p1 = GetValue(t1, id);
      Eigen::Vector2d p2 = GetValue(t2, id);
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
    if (parallaxes.empty()) { return 0.0; }
    std::sort(parallaxes.begin(), parallaxes.end());
    return parallaxes[parallaxes.size() / 2];
  } else {
    if (num_correspondences == 0.0) { return 0.0; }
    return total_parallax / num_correspondences;
  }
}

const std::set<uint64_t>& LandmarkContainer::GetMeasurementTimes() const {
  return measurement_times_;
}

const std::vector<uint64_t>
    LandmarkContainer::GetMeasurementTimesVector() const {
  std::vector<uint64_t> img_times;
  std::for_each(measurement_times_.begin(), measurement_times_.end(),
                [&](const uint64_t& time) { img_times.push_back(time); });
  return img_times;
}

TimeType LandmarkContainer::FrontTimestamp() const {
  const auto first_time = *(measurement_times_.begin());
  return beam::NSecToRos(first_time);
}

TimeType LandmarkContainer::BackTimestamp() const {
  const auto last_time = *(measurement_times_.rbegin());
  return beam::NSecToRos(last_time);
}

void LandmarkContainer::PopFront() {
  RemoveMeasurementsAtTime(FrontTimestamp());
}

void LandmarkContainer::PopBack() {
  RemoveMeasurementsAtTime(BackTimestamp());
}

size_t LandmarkContainer::NumImages() const {
  return measurement_times_.size();
}

void LandmarkContainer::SaveToJson(const std::string& output_filename) {
  if (boost::filesystem::exists(output_filename)) {
    BEAM_WARN("Overriding landmarks in: {}", output_filename);
  }

  if (boost::filesystem::extension(output_filename) != ".json") {
    BEAM_ERROR("Output filename does not have a .json extension, not loading "
               "landmarks. Input: {}",
               output_filename);
    return;
  }

  // create json
  nlohmann::json J;

  // Use the index sorted by landmark id
  const auto& landmark_index =
      this->storage.template get<typename landmark_container::landmark_index>();
  auto unique_ids = std::vector<LandmarkIdType>{};
  // Iterate over all measurements sorted by time first, then landmark_id.
  // Copy landmark ids into a vector, skipping consecutive equal elements
  for (const auto meas : landmark_index) {
    if (unique_ids.empty() || meas.landmark_id != unique_ids.back()) {
      unique_ids.push_back(meas.landmark_id);
      J[std::to_string(meas.landmark_id)] = meas.ToJson();
    }
  }

  std::ofstream file(output_filename);
  file << std::setw(4) << J << std::endl;
}

bool LandmarkContainer::LoadFromJson(const std::string& input_filename,
                                     bool output_info) {
  nlohmann::json J;
  beam::JsonReadErrorType error_type;
  if (!beam::ReadJson(input_filename, J, error_type, output_info)) {
    return false;
  }

  if (output_info) { BEAM_INFO("Loading landmarks from {}", input_filename); }

  std::map<std::string, nlohmann::json> landmark_json_map = J;
  for (auto iter = landmark_json_map.begin(); iter != landmark_json_map.end();
       iter++) {
    MeasurementType meas(iter->second);
    this->composite().insert(meas);
  }

  return true;
}

landmark_container_iterator LandmarkContainer::begin() {
  return this->composite().begin();
}

landmark_container_iterator LandmarkContainer::end() {
  return this->composite().end();
}

const_landmark_container_iterator LandmarkContainer::begin() const {
  return this->composite().begin();
}

const_landmark_container_iterator LandmarkContainer::end() const {
  return this->composite().end();
}

const_landmark_container_iterator LandmarkContainer::cbegin() const {
  return this->composite().cbegin();
}

const_landmark_container_iterator LandmarkContainer::cend() const {
  return this->composite().cend();
}

composite_type& LandmarkContainer::composite() {
  return this->storage
      .template get<typename landmark_container::composite_index>();
}

const composite_type& LandmarkContainer::composite() const {
  return this->storage
      .template get<typename landmark_container::composite_index>();
}

} // namespace beam_containers