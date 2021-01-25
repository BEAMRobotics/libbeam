#include <beam_utils/filesystem.h>

#include <iostream>

#include <boost/filesystem.hpp>

namespace beam {

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

} // namespace beam
