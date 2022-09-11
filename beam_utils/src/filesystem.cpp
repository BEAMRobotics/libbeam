#include <beam_utils/filesystem.h>

#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>

namespace beam {

std::string GetExtension(const std::string& input) {
  return boost::filesystem::extension(input);
}

bool HasExtension(const std::string& input, const std::string& extension) {
  // get extension
  std::string extension_found = GetExtension(input);

  // convert both to lowercase
  std::string extension_search_lowercase = extension;
  std::for_each(extension_search_lowercase.begin(),
                extension_search_lowercase.end(),
                [](char& c) { c = ::tolower(c); });
  std::string extension_found_lowercase = extension_found;
  std::for_each(extension_found_lowercase.begin(),
                extension_found_lowercase.end(),
                [](char& c) { c = ::tolower(c); });

  return extension_found_lowercase == extension_search_lowercase;
}

std::string LibbeamRoot() {
  std::string path_from_root = "beam_utils/src/filesystem.cpp";
  std::string root_location = __FILE__;
  root_location.erase(root_location.end() - path_from_root.length(),
                      root_location.end());
  return root_location;
}

std::string CleanupPath(const std::string& path) {
  namespace fs = boost::filesystem;
  if (path.size() < 3) { return path; }
  std::string return_path;
  for (uint32_t i = 0; i < path.size() - 1; i++) {
    return_path += path[i];
    if (path[i] == fs::path::preferred_separator &&
        path[i + 1] == fs::path::preferred_separator) {
      i++;
    }
  }

  // add last character only if not a file separator
  if (path[path.size() - 1] != fs::path::preferred_separator) {
    return_path += path[path.size() - 1];
  }
  return return_path;
}
std::string CombinePaths(const std::string& path1, const std::string& path2) {
  namespace fs = boost::filesystem;
  fs::path p1(path1);
  fs::path p2(path2);
  fs::path p = p1 / p2;
  return CleanupPath(p.string());
}

std::string CombinePaths(const std::vector<std::string>& paths) {
  namespace fs = boost::filesystem;
  fs::path p;
  for (uint32_t i = 0; i < paths.size(); i++) {
    fs::path p_add(paths[i]);
    p = p / p_add;
  }
  return CleanupPath(p.string());
}

std::vector<std::string> GetFiles(const std::string& directory,
                                  const std::string& extension,
                                  bool recursive) {
  if (!boost::filesystem::is_directory(directory)) {
    BEAM_ERROR("Directory does not exist: {}", directory);
    return std::vector<std::string>{};
  }

  std::vector<std::string> paths{};
  if (recursive) {
    for (auto& p : boost::filesystem::recursive_directory_iterator(directory)) {
      if (p.path().extension().string().empty()) { continue; }
      if (extension.empty() || p.path().extension() == extension) {
        paths.push_back(p.path().string());
      }
    }
  } else {
    for (auto& p : boost::filesystem::directory_iterator(directory)) {
      if (p.path().extension().string().empty()) { continue; }
      if (extension.empty() || p.path().extension() == extension) {
        paths.push_back(p.path().string());
      }
    }
  }
  return paths;
}

nlohmann::json TransformToJson(const Eigen::Matrix4d& T,
                               const std::string& name) {
  nlohmann::json J = {{name,
                       {T(0, 0), T(0, 1), T(0, 2), T(0, 3), T(1, 0), T(1, 1),
                        T(1, 2), T(1, 3), T(2, 0), T(2, 1), T(2, 2), T(2, 3),
                        T(3, 0), T(3, 1), T(3, 2), T(3, 3)}}};
  return J;
}

void AddTransformToJson(nlohmann::json& J, const Eigen::Matrix4d& T,
                        const std::string& name) {
  J.update(TransformToJson(T, name));
}

nlohmann::json ToJsonPoseObject(uint64_t t, const Eigen::Matrix4d& T) {
  nlohmann::json pose_object = {
      {"nsecs", t},
      {"T",
       {T(0, 0), T(0, 1), T(0, 2), T(0, 3), T(1, 0), T(1, 1), T(1, 2), T(1, 3),
        T(2, 0), T(2, 1), T(2, 2), T(2, 3), T(3, 0), T(3, 1), T(3, 2),
        T(3, 3)}}};
  return pose_object;
}

void AddPoseToJson(nlohmann::json& J, uint64_t t, const Eigen::Matrix4d& T) {
  J[std::to_string(t)] = {T(0, 0), T(0, 1), T(0, 2), T(0, 3), T(1, 0), T(1, 1),
                          T(1, 2), T(1, 3), T(2, 0), T(2, 1), T(2, 2), T(2, 3),
                          T(3, 0), T(3, 1), T(3, 2), T(3, 3)};
}

bool ReadJson(const std::string& filename, nlohmann::json& J,
              JsonReadErrorType& error_type, bool output_error) {
  // check file exists
  if (!boost::filesystem::exists(filename)) {
    if (output_error) {
      BEAM_ERROR("CheckJson failed - Json file does not exist. Input: {}",
                 filename);
    }
    error_type = JsonReadErrorType::MISSING;
    return false;
  }

  // check for correct file extension
  if (!HasExtension(filename, ".json")) {
    if (output_error) {
      BEAM_ERROR("CheckJson failed - Invalid file extension. Input: {}",
                 filename);
    }
    error_type = JsonReadErrorType::FILETYPE;
    return false;
  }

  // load json and check that it's not empty
  std::ifstream file(filename);
  file >> J;

  if (J.empty()) {
    if (output_error) {
      BEAM_ERROR("CheckJson failed - Json file is empty. Input: {}", filename);
    }
    error_type = JsonReadErrorType::EMPTY;
    return false;
  }

  error_type = JsonReadErrorType::NONE;
  return true;
}

bool StringToNumericValues(const std::string& deliminator,
                           std::string& input_string,
                           std::vector<double>& values) {
  values.clear();
  size_t pos = 0;

  if ((pos = input_string.find(deliminator)) == std::string::npos) {
    BEAM_ERROR(
        "Input string deliminator is not {}. Input vector remains empty.",
        deliminator);
    return false;
  }

  input_string += deliminator;
  while ((pos = input_string.find(deliminator)) != std::string::npos) {
    double val = std::stod(input_string.substr(0, pos));
    input_string.erase(0, pos + deliminator.length());
    values.push_back(val);
  }

  return true;
}

} // namespace beam
