#pragma once

#include <boost/multi_index/composite_key.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index_container.hpp>
#include <boost/version.hpp>

namespace beam_containers {

/** Internal implementation details - for developers only */
namespace internal {

using boost::multi_index::composite_key;
using boost::multi_index::indexed_by;
using boost::multi_index::member;
using boost::multi_index::ordered_non_unique;
using boost::multi_index::ordered_unique;
using boost::multi_index::tag;

/** Holds all the type definitions required for a boost::multi_index_container
 * holding measurements of type T. See `wave::internal::measurement_container`.
 */
template <typename T>
struct landmark_container {
  // First, define which members of the Measurement object are used as keys
  struct time_key : member<T, decltype(T::time_point), &T::time_point> {};
  struct sensor_key : member<T, decltype(T::sensor_id), &T::sensor_id> {};
  struct landmark_key : member<T, decltype(T::landmark_id), &T::landmark_id> {};

  // Define a composite key which combines the above three keys
  // This lets us search elements sorted by time, then sensor id, then
  // landmark ID
  struct combined_key : composite_key<T, time_key, sensor_key, landmark_key> {};
  struct sensor_composite_key
      : composite_key<T, sensor_key, time_key, landmark_key> {};

  // These types are used as tags to retrieve each index after the
  // multi_index_container is generated
  struct time_index {};
  struct sensor_index {};
  struct sensor_composite_index {};
  struct landmark_index {};
  struct composite_index {};

  // Define an index for each key. Each index will be accessible via its tag
  struct indices
      : indexed_by<
            ordered_non_unique<tag<time_index>, time_key>,
            ordered_non_unique<tag<sensor_index>, sensor_key>,
            ordered_non_unique<tag<landmark_index>, landmark_key>,
            ordered_unique<tag<composite_index>, combined_key>,
            ordered_unique<tag<sensor_composite_index>, sensor_composite_key>> {
  };

  // Finally, define the multi_index_container type.
  // This is the container type which can actually be used to make objects
  using type = boost::multi_index_container<T, indices, std::allocator<T>>;
  // For convenience, get the type of the indices, using their tags
  using composite_type = typename type::template index<composite_index>::type;
  using time_type = typename type::template index<time_index>::type;
  using sensor_composite_type =
      typename type::template index<sensor_composite_index>::type;
  using landmark_type = typename type::template index<landmark_index>::type;
  // Define a view indexed by time, for complex searches
  struct const_time_index
      : indexed_by<ordered_non_unique<
            member<T, const decltype(T::time_point), &T::time_point>>> {};
  using time_view = boost::multi_index_container<const T*, const_time_index>;
};
} // namespace internal

/** Container which stores landmark measurements.
 * @tparam T is the stored measurement type. The `LandmarkMeasurement` class
 * template is designed to be used here.
 *
 * However, any class can be used that has the following public members:
 *   - `time_point` (any sortable type)
 *   - `sensor_id` (any sortable type)
 *   - `landmark_id` (any sortable type)
 *   - `value` (any type)
 *
 * A type is sortable if it can be compared by `std::less`.
 */
template <typename T>
class LandmarkContainer {
public:
  // Types

  /** Alias for the template parameter, giving the type of measurement stored
   * in this container */
  using MeasurementType = T;
  /** Alias for the measurement's time type */
  using TimeType = decltype(MeasurementType::time_point);
  /** Alias for the measurement's value type.
   * Note this does *not* correspond to a typical container's value_type. */
  using ValueType = decltype(MeasurementType::value);
  /** Alias for the type of the sensor id */
  using SensorIdType = decltype(MeasurementType::sensor_id);
  /** Alias for the type of the landmark id */
  using LandmarkIdType = decltype(MeasurementType::landmark_id);
  /** A vector representing landmark / feature measurements across images */
  using Track = std::vector<MeasurementType>;

  using iterator =
      typename internal::landmark_container<T>::composite_type::iterator;
  using const_iterator =
      typename internal::landmark_container<T>::composite_type::const_iterator;
  using sensor_iterator =
      typename internal::landmark_container<T>::sensor_composite_type::iterator;

  // Constructors

  /** Default construct an empty container */
  LandmarkContainer() {}

  /** Construct the container with the contents of the range [first, last) */
  template <typename InputIt>
  LandmarkContainer(InputIt first, InputIt last) {
    this->composite().insert(first, last);
  };

  // Capacity

  /** Return true if the container has no elements. */
  bool empty() const { return this->composite().empty(); }

  /** Return the number of elements in the container. */
  size_t size() const { return this->composite().size(); }

  // Modifiers

  /** @brief Insert a Measurement if a measurement for the same time and sensor
   * does not already exist.
   * @return a pair p. If and only if insertion occurred, p.second is true and
   * p.first points to the element inserted.
   */
  std::pair<iterator, bool> Insert(const MeasurementType& m) {
    return this->composite().insert(m);
  }

  /** @brief For each element of the range [first, last), inserts a Measurement
   * if a measurement for the same time and sensor does not already exist.
   * @param first, last iterators representing a valid range of Measurements,
   * but not iterators into this container
   */
  template <typename InputIt>
  void Insert(InputIt first, InputIt last) {
    return this->composite().insert(first, last);
  }

  /** @brief Insert a Measurement constructed from the arguments if a
   * measurement for the same time and sensor does not already exist.
   * @return a pair p. If and only if insertion occurred, p.second is true and
   * p.first points to the element inserted.
   */
  template <typename... Args>
  std::pair<iterator, bool> Emplace(Args&&... args) {
    return this->composite().emplace(std::forward<Args>(args)...);
  }

  /** @brief Delete the element with the matching time, sensor, and landmark id
   * if one exists.
   * @return the number of elements deleted.
   */
  size_t Erase(const TimeType& t, SensorIdType s, LandmarkIdType id) {
    auto& composite = this->composite();
    auto it = composite.find(boost::make_tuple(t, s, id));
    if (it == composite.end()) { return 0; }
    composite.erase(it);
    return 1;
  }

  /** Delete the element at `position`
   * @param position a valid dereferenceable iterator of this container
   * @return An iterator pointing to the element following the deleted one, or
   * `end()` if it was the last.
   */
  iterator Erase(iterator position) {
    return this->composite().erase(position);
  }

  /** @brief Delete the elements in the range [first, last)
   * @param first, last a valid range of this container
   * @return `last`
   */
  iterator Erase(iterator first, iterator last) {
    return this->composite().erase(first, last);
  }

  /** Delete all elements */
  void clear() { return this->composite().clear(); }

  // Retrieval

  /** @briefGets the value of a landmark measurement.
   * @throw std::out_of_range if a measurement with exactly matching time,
   * sensor, and landmark id does not exist.
   * No interpolation is performed.
   */
  ValueType Get(const TimeType& t, SensorIdType s, LandmarkIdType id) const {
    const auto& composite = this->composite();
    auto iter = composite.find(boost::make_tuple(t, s, id));
    if (iter == composite.end()) {
      // Requested key is not in this container
      throw std::out_of_range("LandmarkContainer::get");
    }
    return iter->value;
  }

  /** @brief Get all measurements from the given sensor
   * @return a pair of iterators representing the start and end of the range.
   * If the range is empty, both iterators will be equal.
   * @note because these iterators use the underlying ordered index of
   * sensor_ids, they are not the same type as those from `begin()`,
   * `getTimeWindow()`, etc.
   */
  std::pair<sensor_iterator, sensor_iterator>
      GetAllFromSensor(const SensorIdType& s) const {
    // Get the measurements sorted by sensor_id
    const auto& sensor_composite_index = this->storage.template get<
        typename internal::landmark_container<T>::sensor_composite_index>();
    return sensor_composite_index.equal_range(s);
  };

  /** @brief Get all measurements between the given times.
   * @param start, end an inclusive range of times, with start <= end
   * @return a pair of iterators representing the start and end of the range.
   * If the range is empty, both iterators will be equal.
   */
  std::pair<iterator, iterator> GetTimeWindow(const TimeType& start,
                                              const TimeType& end) const {
    // Consider a "backward" window empty
    if (start > end) { return {this->end(), this->end()}; }
    // The composite index is already sorted by time first, thus it's enough to
    // do a partial search. Find the start and end of the range.
    const auto& composite = this->composite();
    auto iter_begin = composite.lower_bound(boost::make_tuple(start));
    auto iter_end = composite.upper_bound(boost::make_tuple(end));
    return {iter_begin, iter_end};
  }

  /** Get a list of all unique landmark IDs in the container */
  std::vector<LandmarkIdType> GetLandmarkIDs() const {
    return this->GetLandmarkIDsInWindow(TimeType::min(), TimeType::max());
  }

  /** @brief Get unique landmark IDs with measurements in the time window
   * @return a vector of landmark IDs, in increasing order
   * The window is inclusive. If `start > end`, the result will be empty.
   */
  std::vector<LandmarkIdType>
      GetLandmarkIDsInWindow(const TimeType& start, const TimeType& end) const {
    // Use the index sorted by landmark id
    const auto& landmark_index = this->storage.template get<
        typename internal::landmark_container<T>::landmark_index>();
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

  /** @briefGet a sequence of measurements of a landmark from one sensor
   * @return a vector of landmark measurements sorted by time
   */
  Track GetTrack(const SensorIdType& s, const LandmarkIdType& id) const {
    return this->GetTrackInWindow(s, id, TimeType::min(), TimeType::max());
  };

  /** @brief Get a sequence of measurements of a landmark from one sensor, in
   * the given time window.
   * @return a vector of landmark measurements sorted by time
   * The window is inclusive. If `start > end`, the result will be empty.
   */
  Track GetTrackInWindow(const SensorIdType& s, const LandmarkIdType& id,
                         const TimeType& start, const TimeType& end) const {
    // Consider a "backwards" window empty
    if (start > end) { return Track{}; }
    const auto& landmark_index = this->storage.template get<
        typename internal::landmark_container<T>::landmark_index>();
    // Get all measurements with desired landmark id
    const auto res = landmark_index.equal_range(id);
    // We need to get a track sorted by time, but landmark_index is not.
    // Construct a view, indexed by time, with only those measurements
    // This method is based on one described here:
    // http://www.boost.org/doc/libs/1_63_0/libs/multi_index/doc/examples.html#example6
    // While iterating, pick the measurements with desired sensor_id
    auto time_view = typename internal::landmark_container<T>::time_view{};
    for (auto it = res.first; it != res.second; ++it) {
      if (it->sensor_id == s) {
        // insert a pointer to the measurement
        time_view.insert(&*it);
      }
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

  // Iterators

  iterator begin() { return this->composite().begin(); }
  iterator end() { return this->composite().end(); }
  const_iterator begin() const { return this->composite().begin(); }
  const_iterator end() const { return this->composite().end(); }
  const_iterator cbegin() const { return this->composite().cbegin(); }
  const_iterator cend() const { return this->composite().cend(); }

protected:
  using composite_type =
      typename internal::landmark_container<T>::composite_type;

  // Helper to get the composite index
  composite_type& composite() {
    return this->storage.template get<
        typename internal::landmark_container<T>::composite_index>();
  }

  const composite_type& composite() const {
    return this->storage.template get<
        typename internal::landmark_container<T>::composite_index>();
  }

  // Internal multi_index_container
  typename internal::landmark_container<T>::type storage;
};

} // namespace beam_containers
