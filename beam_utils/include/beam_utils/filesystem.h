/** @file
 * @ingroup utils
 *
 * File system utils for finding files or paths to files, etc.
 *
 */

#pragma once

#include <beam_utils/log.h>

namespace beam {
/** @addtogroup utils
 *  @{ */

/**
 * @brief get the path to libbeam's root folder
 */
std::string LibbeamRoot();

/**
 * @brief get a vector of absolute paths to files in a directory
 * @param directory absolute path to directory to search in
 * @param extension file extensions to find
 * @param recursive set to true to search for files recursively in directory
 * @return list of strings with absolute paths to all files found
 */
std::vector<std::string> GetFiles(const std::string& directory,
                                  const std::string& extension = "",
                                  bool recursive = false);

/** @} group utils */
} // namespace beam

