/** @file
 * @ingroup utils
 *
 * File system utils for finding files or paths to files, etc.
 *
 */

#pragma once

#include <beam_utils/log.h>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>

namespace beam {
/** @addtogroup utils
 *  @{ */

/**
 * @brief returns true if the input string finishes with the extension specified by entension (e.g., ".json")
 */
bool HasExtension(const std::string& input, const std::string& extension);


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

nlohmann::json TransformToJson(const Eigen::Matrix4d& T, const std::string& name);

void AddTransformToJson(nlohmann::json& J, const Eigen::Matrix4d& T, const std::string& name);

nlohmann::json ToJsonPoseObject(uint64_t t, const Eigen::Matrix4d& T);

void AddPoseToJson(nlohmann::json& J, uint64_t t, const Eigen::Matrix4d& T);

/** @} group utils */
} // namespace beam

