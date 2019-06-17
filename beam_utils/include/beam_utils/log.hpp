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

#ifndef BEAM_UTILS_LOG_HPP
#define BEAM_UTILS_LOG_HPP

#ifndef NDEBUG
  #define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif
#include "spdlog/spdlog.h"

namespace beam {
/** @addtogroup utils
 *  @{ */

#define FILENAME \
    (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define LOG_ERROR(M, ...) \
    fprintf(              \
      stderr, "[ERROR] [%s:%d] " M "\n", FILENAME, __LINE__, ##__VA_ARGS__)

#define LOG_INFO(M, ...) fprintf(stdout, "[INFO] " M "\n", ##__VA_ARGS__)

#define LOG_WARNING(M, ...) fprintf(stdout, "[WARNING] " M "\n", ##__VA_ARGS__)

/** @} group utils */
}  // namespace beam

#endif  // BEAM_UTILS_LOGGING_HPP
