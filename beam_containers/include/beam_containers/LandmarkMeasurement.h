#pragma once

#include <chrono>
#include <beam_utils/math.hpp>

namespace beam_containers {
/** @addtogroup containers
 *  @{ */

/** The integral type used to track a landmark across multiple measurements
 */
using ImageNum = std::size_t;
using LandmarkId = std::size_t;
using TimePoint = std::chrono::steady_clock::time_point;

/** Storage type for a landmark measurement as a 2D position.
 *
 * Presumably the landmark measurement has been extracted from a camera image,
 * but it is now a separate entity with no record of which camera image it came
 * from.
 *
 * This class template is similar to `Measurement`, with an additional member:
 * `landmark_id`, which represents a unique ID used for tracking a single
 * landmark across multiple images.
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
    LandmarkId landmark_id;
    ImageNum image;
    beam::Vec2 value;

    LandmarkMeasurement(const TimePoint &t,
                        const SensorIdType &s,
                        const LandmarkId &id,
                        const ImageNum &img,
                        const beam::Vec2 &v)
        : time_point{t}, sensor_id{s}, landmark_id{id}, image{img}, value{v} {}
};


}  // namespace beam_containers
