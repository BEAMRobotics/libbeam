/** @file
 * @ingroup utils
 *
 * File system utils for finding files or paths to files, etc.
 *
 */

#pragma once

#include <Eigen/Dense>
#include <beam_utils/log.h>
#include <nlohmann/json.hpp>

namespace beam {
/** @addtogroup utils
 *  @{ */

/**
 * @brief enum class for storing error types for the CheckJson function.
 *
 *  NONE: no error
 *  MISSING: file does not exist
 *  FILETYPE: file extension is not .json
 *  EMPTY: json file is empty
 *
 */
enum JsonReadErrorType { NONE, FILETYPE, MISSING, EMPTY };

static JsonReadErrorType tmp_json_read_error_type_ = JsonReadErrorType::NONE;

/**
 * @brief method for getting a string of the file extension, e.g.: ".json". If
 * there is no extension, the string will be empty
 * @param input filename which can be a path
 * @return extension
 */
std::string GetExtension(const std::string& input);

/**
 * @brief returns true if the input string finishes with the extension specified
 * by entension (e.g., ".json"). This will also convert both strings to lower
 * case
 */
bool HasExtension(const std::string& input, const std::string& extension);

/**
 * @brief get the path to libbeam's root folder
 */
std::string LibbeamRoot();

/**
 * @brief remove duplicate file separators in a string filepath
 *
 * @param path
 * @return std::string
 */
std::string RemoveDuplicateFileSeparators(const std::string& path);

/**
 * @brief safe way (for all OSs) of combining paths. Uses std::filesystem
 *
 * @param path1 first path to add
 * @param path2 second path to add
 * @return std::string
 */
std::string CombinePaths(const std::string& path1, const std::string& path2);

/**
 * @brief safe way (for all OSs) of combining paths. Uses std::filesystem
 *
 * @param paths vector of paths to combine in order
 * @return std::string
 */
std::string CombinePaths(const std::vector<std::string>& paths);

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

nlohmann::json TransformToJson(const Eigen::Matrix4d& T,
                               const std::string& name);

void AddTransformToJson(nlohmann::json& J, const Eigen::Matrix4d& T,
                        const std::string& name);

nlohmann::json ToJsonPoseObject(uint64_t t, const Eigen::Matrix4d& T);

void AddPoseToJson(nlohmann::json& J, uint64_t t, const Eigen::Matrix4d& T);

/**
 * @brief reads a json and does some checks:
 *
 *  1. Check that file exists
 *  2. Check that file has the .json extension
 *  3. Check the json is not null
 *
 * @param filename full path to json
 * @param J reference to json to fill
 * @param error_type optional reference to enum class of error type
 * @param output_error optional bool to set if you want this function to output
 * the error type
 * @return true if passed
 */
bool ReadJson(const std::string& filename, nlohmann::json& J,
              JsonReadErrorType& error_type = tmp_json_read_error_type_,
              bool output_error = true);

/**
 * @brief converts tokens, separated by a common deliminator, from an input
 * string into a vector of numeric values
 * @param deliminator deliminator separating tokens
 * @param input_string input string to parse
 * @param values vector containing tokens as numeric values
 * @return true if values is non-empty
 */
bool StringToNumericValues(const std::string& deliminator,
                           std::string& input_string,
                           std::vector<double>& values);

void ValidateJsonKeysOrThrow(const std::vector<std::string>& required_keys,
                             const nlohmann::json& J);

/** @} group utils */
} // namespace beam
