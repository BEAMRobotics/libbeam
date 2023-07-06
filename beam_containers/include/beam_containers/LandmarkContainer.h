#pragma once

#include <fstream>
#include <iomanip>
#include <map>
#include <set>

#include <boost/filesystem.hpp>
#include <boost/multi_index/composite_key.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index_container.hpp>
#include <boost/version.hpp>
#include <nlohmann/json.hpp>

#include <beam_containers/LandmarkMeasurement.h>
#include <beam_utils/filesystem.h>
#include <beam_utils/log.h>

namespace beam_containers {

/** Internal implementation details - for developers only */
namespace internal {

using boost::multi_index::composite_key;
using boost::multi_index::indexed_by;
using boost::multi_index::member;
using boost::multi_index::ordered_non_unique;
using boost::multi_index::ordered_unique;
using boost::multi_index::tag;

/**
 * Holds all the type definitions required for a boost::multi_index_container
 * holding measurements of type T. See `wave::internal::measurement_container`.
 */
template <class T>
struct landmark_container {
  using time_t = decltype(T::time_point);
  using id_t = decltype(T::landmark_id);
  // Keys being used in this composite index
  struct time_key : member<T, time_t, &T::time_point> {};
  struct landmark_key : member<T, id_t, &T::landmark_id> {};
  struct combined_key : composite_key<T, time_key, landmark_key> {};

  // These types are used as tags to retrieve each index after the
  // multi_index_container is generated
  struct time_index {};
  struct landmark_index {};
  struct composite_index {};

  // Define an index for each key. Each index will be accessible via its tag
  struct indices
      : indexed_by<ordered_non_unique<tag<time_index>, time_key>,
                   ordered_non_unique<tag<landmark_index>, landmark_key>,
                   ordered_unique<tag<composite_index>, combined_key>> {};

  // Finally, define the multi_index_container type.
  // This is the container type which can actually be used to make objects
  using type = boost::multi_index_container<T, indices, std::allocator<T>>;
  // For convenience, get the type of the indices, using their tags
  using composite_type = typename type::template index<composite_index>::type;
  using time_type = typename type::template index<time_index>::type;
  using landmark_type = typename type::template index<landmark_index>::type;
  // Define a view indexed by time, for complex searches
  struct const_time_index
      : indexed_by<
            ordered_non_unique<member<T, const time_t, &T::time_point>>> {};
  using time_view = boost::multi_index_container<const T*, const_time_index>;
};
} // namespace internal

// Types

using MeasurementType = beam_containers::LandmarkMeasurement;
using TimeType = decltype(MeasurementType::time_point);
using ValueType = decltype(MeasurementType::value);
using LandmarkIdType = decltype(MeasurementType::landmark_id);
using Track = std::vector<MeasurementType>;

using landmark_container = internal::landmark_container<MeasurementType>;
using landmark_container_iterator =
    landmark_container::composite_type::iterator;
using const_landmark_container_iterator =
    typename landmark_container::composite_type::const_iterator;
using composite_type = typename landmark_container::composite_type;

/**
 * @brief Container which stores  beam_containers::LandmarkMeasurement's.
 */
class LandmarkContainer {
public:
  /**
   * @brief Default construct an empty container
   */
  LandmarkContainer() {}

  /**
   * @brief Return true if the container has no elements.
   */
  bool empty() const;

  /**
   * @brief Return the number of elements in the container.
   */
  size_t size() const;

  /**
   * @brief Delete all elements
   */
  void clear();

  /**
   * @brief Insert a Measurement if a measurement for the same time and sensor
   * does not already exist.
   * @return success or not
   */
  bool Insert(const MeasurementType& m);

  /**
   * @brief Delete the element with the matching time, sensor, and landmark id
   * if one exists.
   * @return pass or fail
   */
  bool Erase(const TimeType& t, const LandmarkIdType id);

  /**
   * @brief Gets the value of a landmark measurement.
   * @throw std::out_of_range if a measurement with exactly matching time,
   * sensor, and landmark id does not exist.
   * No interpolation is performed.
   * @param t timestamp
   * @param id landmark id
   * @return value in measurement object
   */
  ValueType GetValue(const TimeType& t, LandmarkIdType id) const;

  /**
   * @brief Gets the full measurement from a time point, sensor id and landmark
   * id
   * @throw std::out_of_range if a measurement with exactly matching time,
   * sensor, and landmark id does not exist.
   * No interpolation is performed.
   * @param t timestamp
   * @param id landmark id
   * @return measurement object
   */
  MeasurementType GetMeasurement(const TimeType& t, LandmarkIdType id) const;

  /**
   * @brief Get all measurements between the given times.
   * @param start, start of an inclusive range of times, with start <= end
   * @param end, end of an inclusive range of times, with start <= end
   * @return a pair of iterators representing the start and end of the range.
   * If the range is empty, both iterators will be equal.
   */
  std::pair<landmark_container_iterator, landmark_container_iterator>
      GetTimeWindow(const TimeType& start, const TimeType& end) const;

  /**
   * @brief Get a list of all unique landmark IDs in the container
   */
  std::vector<LandmarkIdType> GetLandmarkIDs() const;

  /**
   * @brief Get unique landmark IDs with measurements in the time window
   * @param start, start of an inclusive range of times, with start <= end
   * @param end, end of an inclusive range of times, with start <= end
   * @return a vector of landmark IDs, in increasing order
   * The window is inclusive. If `start > end`, the result will be empty.
   */
  std::vector<LandmarkIdType> GetLandmarkIDsInWindow(const TimeType& start,
                                                     const TimeType& end) const;

  /**
   * @brief Get unique landmark IDs with measurements in the specific image
   * @param img_time time of image to get id's for
   * @return a vector of landmark IDs, in increasing order
   */
  std::vector<LandmarkIdType>
      GetLandmarkIDsInImage(const TimeType& img_time) const;

  /**
   * @brief Get a sequence of measurements of a landmark from one sensor
   * @param id id of landmark to get track for
   * @return a vector of landmark measurements sorted by time
   */
  Track GetTrack(const LandmarkIdType& id) const;

  /**
   * @brief Get a sequence of measurements of a landmark from one sensor, in
   * the given time window.
   * @param id landmark id to get track for
   * @param start, start of an inclusive range of times, with start <= end
   * @param end, end of an inclusive range of times, with start <= end
   * @return a vector of landmark measurements sorted by time
   * The window is inclusive. If `start > end`, the result will be empty.
   */
  Track GetTrackInWindow(const LandmarkIdType& id, const TimeType& start,
                         const TimeType& end) const;

  /**
   * @brief Remove all landmark measurements at a specified time
   * @param time image time to remove measurements for
   */
  void RemoveMeasurementsAtTime(const TimeType& time);

  /**
   * @brief Computes the parallax fo measurements between two times
   * @param t1 first time
   * @param t2 second time
   */
  double ComputeParallax(const TimeType& t1, const TimeType& t2,
                         bool compute_median = false);

  /**
   * @brief Return a const reference to the measurement times set
   */
  const std::set<ros::Time>& GetMeasurementTimes() const;

  /**
   * @brief Return an ordered vector of measurement times
   */
  const std::vector<ros::Time> GetMeasurementTimesVector() const;

  /**
   * @brief Return the timestamp at the start of the container
   */
  TimeType FrontTimestamp() const;

  /**
   * @brief  Return the timestamp at the end of the container
   */
  TimeType BackTimestamp() const;

  /**
   * @brief Remove first image from the container
   */
  void PopFront();

  /**
   * @brief Remove last image from the container
   */
  void PopBack();

  /**
   * @brief Retrun the number of images in the container
   */
  size_t NumImages() const;

  /**
   * @brief save all measurements in container to disk as json
   * @param output_filename full path to output dir + filename. The directory of
   * this output file must exist.
   */
  void SaveToJson(const std::string& output_filename);

  /**
   * @brief load measurements from a json into container.
   * @param input_filename full path to input json file
   * @param output_info outputs read file location and any errors that occur
   * @return true if successful
   */
  bool LoadFromJson(const std::string& input_filename, bool output_info = true);

  // Iterators
  landmark_container_iterator begin();
  landmark_container_iterator end();
  const_landmark_container_iterator begin() const;
  const_landmark_container_iterator end() const;
  const_landmark_container_iterator cbegin() const;
  const_landmark_container_iterator cend() const;

protected:
  composite_type& composite();

  const composite_type& composite() const;

  // Internal multi_index_container
  typename landmark_container::type storage;

  std::set<ros::Time> measurement_times_;
};

} // namespace beam_containers
