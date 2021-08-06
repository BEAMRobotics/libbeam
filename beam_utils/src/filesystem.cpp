#include <beam_utils/filesystem.h>

#include <iostream>

#include <boost/filesystem.hpp>

namespace beam {

bool HasExtension(const std::string& input, const std::string& extension) {
  std::string file_extension = input;
  file_extension.erase(file_extension.end() - input.length(),
                       file_extension.end() - extension.length());
  if (file_extension != extension) { return false; }
  return true;
}

std::string LibbeamRoot() {
  std::string path_from_root = "beam_utils/src/filesystem.cpp";
  std::string root_location = __FILE__;
  root_location.erase(root_location.end() - path_from_root.length(),
                      root_location.end());
  return root_location;
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
       {T(0, 0), T(0, 1), T(0, 2), T(0, 3), T(1, 0), T(1, 1), T(1, 2), T(1,
       3),
        T(2, 0), T(2, 1), T(2, 2), T(2, 3), T(3, 0), T(3, 1), T(3, 2),
        T(3, 3)}}};
  return pose_object;
}

void AddPoseToJson(nlohmann::json& J, uint64_t t, const Eigen::Matrix4d& T) {
  J[std::to_string(t)] = {T(0, 0), T(0, 1), T(0, 2), T(0, 3), T(1, 0), T(1, 1),
          T(1, 2), T(1, 3), T(2, 0), T(2, 1), T(2, 2), T(2, 3),
          T(3, 0), T(3, 1), T(3, 2), T(3, 3)};
}

} // namespace beam
