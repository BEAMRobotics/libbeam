/** @file
 * @ingroup utils
 *
 * Timing functions, useful for measuring how long a particular function
 * executed, etc.
 */

#ifndef BEAM_UTILS_TIME_HPP
#define BEAM_UTILS_TIME_HPP

#include <chrono>
#include <string>
#include <iostream>
#include <ctime>
#include <sys/time.h>
#include <beam/utils/log.hpp>

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
void LogTimePoint(const TimePoint time_point, const std::string output_text) {
  LOG_INFO("%s %f", output_text.c_str(), (double) time_point.time_since_epoch().count());
}

/**
 * @brief Simple way to output the timepoint using cout
 * @param time_point
 * @param output_text
 */
 void OutputTimePoint(const TimePoint time_point, const std::string output_text) {
   std::cout << output_text.c_str()
             << time_point.time_since_epoch().count() << "\n";
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
