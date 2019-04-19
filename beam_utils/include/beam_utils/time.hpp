/** @file
 * @ingroup utils
 *
 * Timing functions, useful for measuring how long a particular function
 * executed, etc.
 */

#ifndef BEAM_UTILS_TIME_HPP
#define BEAM_UTILS_TIME_HPP

#include <beam_utils/log.hpp>
#include <chrono>
#include <ctime>
#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <string>
#include <sys/time.h>

namespace beam {
/** @addtogroup utils
 *  @{ */

typedef std::chrono::steady_clock Clock;
typedef std::chrono::time_point<Clock> TimePoint;

/**
 * @brief Simple way to output the timepoint using LOG_INFO
 * @param time_point
 * @param output_text
 */
inline void LogTimePoint(const TimePoint time_point, const std::string output_text) {
  LOG_INFO("%s %f", output_text.c_str(),
           (double)time_point.time_since_epoch().count());
}

/**
 * @brief Simple way to output the timepoint using cout
 * @param time_point
 * @param output_text
 */
inline void OutputTimePoint(const TimePoint time_point,
                     const std::string output_text) {
  std::cout << output_text.c_str() << time_point.time_since_epoch().count()
            << "\n";
}

/**
 * @brief convert ROS time to a chrono time point
 * @param hdr header from a ROS message
 * @return TimePoint
 */
inline TimePoint rosTimeToChrono(const std_msgs::Header& hdr) {
  std::chrono::seconds secs(hdr.stamp.sec);
  std::chrono::nanoseconds nsecs(hdr.stamp.nsec);
  auto dur = secs + nsecs;
  return TimePoint(dur);
}

/**
 * @brief convert chrono time point to ROS time
 * @param time_point
 * @return ROS time
 */
inline ros::Time chronoToRosTime(const TimePoint& time_point) {
  uint32_t seconds, nanoseconds;
  seconds = std::round(time_point.time_since_epoch().count() / 1000000000);
  double tmp = time_point.time_since_epoch().count() -
               std::round(time_point.time_since_epoch().count());
  nanoseconds = std::round(tmp * 1000000000);
  ros::Time ros_time(seconds, nanoseconds);
  return ros_time;
}

/**
 * @brief Simple timer object
 */
struct HighResolutionTimer {
  HighResolutionTimer() : start_time(take_time_stamp()) {}

  /**
   * @brief Restart timer
   */
  void restart() { start_time = take_time_stamp(); }

  /**
   * @brief Return elapsed time in seconds.
   * @return
   */
  double elapsed() const {
    return double(take_time_stamp() - start_time) * 1e-9;
  }

  std::uint64_t elapsed_nanoseconds() const {
    return take_time_stamp() - start_time;
  }

protected:
  static std::uint64_t take_time_stamp() {
    return std::uint64_t(
        std::chrono::high_resolution_clock::now().time_since_epoch().count());
  }

private:
  std::uint64_t start_time;
};

/** Similar to how matlab's tic and toc work, `tic()` starts the timer where the
 * variable `t` stores the start time, when `toc()` is called it returns the
 * time
 * difference in seconds. `mtoc()` returns the time difference in milli-seconds.
 */
void tic(struct timespec* tic);
float toc(struct timespec* tic);
float mtoc(struct timespec* tic);

/** @return the time since epoch in seconds. */
double time_now(void);

/** @} group utils */
} // namespace beam

#endif // BEAM_UTILS_TIME_HPP
