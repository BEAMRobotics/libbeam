#pragma once

#include <beam_utils/math.h>
#include <beam_utils/time.h>

namespace beam_containers {
/** @addtogroup containers
 *  @{ */

/** The integral type used to track a landmark across multiple measurements
 */
using TimePoint = ros::Time;

/** Storage type for a landmark measurement as a 2D position.
 *
 * Presumably the landmark measurement has been extracted from a camera image,
 * but it is now a separate entity with no record of which camera image it came
 * from.
 *
 * This class represents a visual landmark, and assigns a unique id to each so
 * it can be tracked across various frames
 *
 * The measurement value is 2D vector, representing the pixel position of the
 * landmark (u, v).
 *
 * @tparam SensorIdType The enum or integral type used to identify camera
 * sensors.
 */
template <typename SensorIdType>
struct LandmarkMeasurement {
  TimePoint time_point;
  SensorIdType sensor_id;
  size_t landmark_id;
  size_t image;
  Eigen::Vector2d value;

  LandmarkMeasurement(const TimePoint& t, const SensorIdType& s,
                      const size_t& id, const size_t& img,
                      const Eigen::Vector2d& v)
      : time_point{t}, sensor_id{s}, landmark_id{id}, image{img}, value{v} {}
};

} // namespace beam_containers
