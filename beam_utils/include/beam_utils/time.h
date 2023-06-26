/** @file
 * @ingroup utils
 *
 * Timing functions, useful for measuring how long a particular function
 * executed, etc.
 */

#pragma once

#include <chrono>
#include <ctime>
#include <iostream>
#include <string>
#include <sys/time.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Header.h>

#include <beam_utils/log.h>

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
inline void LogTimePoint(const TimePoint time_point,
                         const std::string output_text) {
  LOG_INFO("%s %f", output_text.c_str(),
           (double)time_point.time_since_epoch().count());
}

/**
 * @brief convert time point to readable string (YYYY_MM_DD_HH_MM_SS). 
 * Example usage:
 * auto date = beam::ConvertTimeToDate(std::chrono::system_clock::now());
 */
inline std::string
    ConvertTimeToDate(std::chrono::system_clock::time_point time_) {
  using namespace std;
  using namespace std::chrono;
  system_clock::duration tp = time_.time_since_epoch();
  time_t tt = system_clock::to_time_t(time_);
  tm local_tm = *localtime(&tt);

  string outputTime =
      to_string(local_tm.tm_year + 1900) + "_" +
      to_string(local_tm.tm_mon + 1) + "_" + to_string(local_tm.tm_mday) + "_" +
      to_string(local_tm.tm_hour) + "_" + to_string(local_tm.tm_min) + "_" +
      to_string(local_tm.tm_sec);
  return outputTime;
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
 * @brief Convert nanoseconds to a ros timestamp
 * @param n_sec
 */
inline ros::Time NSecToRos(const uint64_t n_sec) {
  ros::Time time;
  time.fromNSec(n_sec);
  return time;
}

/**
 * @brief convert ROS time to a chrono time point
 * @param hdr header from a ROS message
 * @return TimePoint
 */
inline TimePoint RosTimeToChrono(const std_msgs::Header& hdr) {
  std::chrono::seconds secs(hdr.stamp.sec);
  std::chrono::nanoseconds nsecs(hdr.stamp.nsec);
  auto dur = secs + nsecs;
  return TimePoint(dur);
}

/**
 * @brief convert ROS time to a chrono time point
 * @param hdr header from a ROS message
 * @return TimePoint
 */
inline TimePoint RosTimeToChrono(const ros::Time& time_in) {
  std::chrono::seconds secs(time_in.sec);
  std::chrono::nanoseconds nsecs(time_in.nsec);
  auto dur = secs + nsecs;
  return TimePoint(dur);
}

/**
 * @brief convert chrono time point to ROS time
 * @param time_point
 * @return ROS time
 */
inline ros::Time ChronoToRosTime(const TimePoint& time_point) {
  uint32_t seconds, nanoseconds;
  seconds = std::round(time_point.time_since_epoch().count() / 1000000000);
  nanoseconds = time_point.time_since_epoch().count() - seconds * 1000000000;
  ros::Time ros_time(seconds, nanoseconds);
  return ros_time;
}

struct RosTimeHash {
  std::size_t operator()(const ros::Time& k) const {
    return std::hash<uint64_t>()(k.toNSec());
  }
};

struct RosTimeCmp {
  bool operator()(const ros::Time& a, const ros::Time& b) const {
    return a < b;
  }
};

template <class ValueType>
using RosTimeMap = std::map<ros::Time, ValueType, RosTimeCmp>;

template <class ValueType>
using RosTimeHashMap = std::unordered_map<ros::Time, ValueType, RosTimeHash>;

using RosTimeSet = std::set<ros::Time, RosTimeCmp>;

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

  /**
   * @brief Return elapsed time in seconds, then restart.
   * @return
   */
  double elapsedAndRestart() {
    double t = double(take_time_stamp() - start_time) * 1e-9;
    start_time = take_time_stamp();
    return t;
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
