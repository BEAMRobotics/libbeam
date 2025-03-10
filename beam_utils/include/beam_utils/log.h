/** @file
 * @ingroup utils
 *
 * Functions to log errors and info to `stderr` and `stdout`.
 *
 * `LOG_ERROR` and `LOG_INFO` both are simple `fprintf()` that can be use to
 * write message `M` to `stderr` and `stdout`. For example:
 * ```
 * LOG_ERROR("Failed to load configuration file [%s]", config_file.c_str());
 * LOG_INFO("Parameter was not found! Loading defaults!");
 * ```
 */

#pragma once

#include <experimental/source_location>
#include <iostream>

#include <boost/filesystem.hpp>

#ifndef NDEBUG
#  define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif
#include "spdlog/spdlog.h"

namespace beam {
/** @addtogroup utils
 *  @{ */

#define FILENAME                                                               \
  (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define LOG_ERROR(M, ...)                                                      \
  fprintf(stderr, "[ERROR] [%s:%d] " M "\n", FILENAME, __LINE__, ##__VA_ARGS__)

#define LOG_INFO(M, ...) fprintf(stdout, "[INFO] " M "\n", ##__VA_ARGS__)

// Helper function for logging the source location of code.
// This will cout: [FL: {FILE}][FN: ${Function}][L: {LINE}][post_fix]
// call using: LogSourceLocation(beam::source_location::current(), "some text to
// add")
using source_location = std::experimental::source_location;
inline void LogSourceLocation(const source_location& source_location,
                              const std::string& post_fix = "") {
  std::string filename =
      boost::filesystem::path(source_location.file_name()).filename().string();
  std::cout << "[F: " << filename
            << "][FNC: " << source_location.function_name()
            << "][L: " << source_location.line() << "][" << post_fix << "]\n";
}

inline void OutputPercentComplete(int current_, int total_,
                                  std::string message_) {
  int out25, out50, out75;
  out25 = round(0.25 * total_);
  out50 = round(0.5 * total_);
  out75 = round(0.75 * total_);

  if (current_ == 1) {
    LOG_INFO("%s", message_.c_str());
    LOG_INFO("0 %% complete (1 of %d) ...", total_);
  } else if (current_ == out25) {
    LOG_INFO("25 %% complete (%d of %d) ...", current_, total_);
  } else if (current_ == out50) {
    LOG_INFO("50 %% complete (%d of %d) ...", current_, total_);
  } else if (current_ == out75) {
    LOG_INFO("75 %% complete (%d of %d) ...", current_, total_);
  } else if (current_ == total_) {
    LOG_INFO("100 %% complete (%d of %d)", current_, total_);
  }
}

template <typename T>
void PrintVector(const std::vector<T>& v) {
  std::cout << "[";
  for (int i = 0; i < v.size() - 1; i++) { std::cout << v[i] << ", "; }
  std::cout << v[v.size() - 1] << "]\n";
}

/** @} group utils */
} // namespace beam
