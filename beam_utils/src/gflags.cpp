#include <beam_utils/gflags.h>

#include <boost/filesystem.hpp>

#include <beam_utils/filesystem.h>

namespace beam { namespace gflags {

bool ValidateCannotBeEmpty(const char* flagname, const std::string& value) {
  if (!value.empty()) { return true; }
  printf("Invalid value for --%s: %s. Cannot be empty.\n", flagname,
         value.c_str());
  return false;
}

bool ValidateDirMustExist(const char* flagname, const std::string& value) { 
  if (value.empty()) {
    printf("Invalid value for --%s: %s. Cannot be empty.\n", flagname,
           value.c_str());
    return false;
  }

  boost::filesystem::path path(value);
  boost::filesystem::path directory = path.parent_path();

  if (!boost::filesystem::exists(directory)) {
    printf("Invalid value for --%s: %s. Directory does not exist.\n", flagname,
           directory.c_str());
    return false;
  }
  return true;
}

bool ValidateFileMustExist(const char* flagname, const std::string& value) {
  if (boost::filesystem::exists(value)) { return true; }
  printf("Invalid value for --%s: %s. File Does not exist.\n", flagname,
         value.c_str());
  return false;
}

bool ValidateJsonFileMustExist(const char* flagname, const std::string& value) {
  if (!boost::filesystem::exists(value)) {
    printf("Invalid value for --%s: %s. File Does not exist.\n", flagname,
           value.c_str());
    return false;
  }

  std::string extension{".json"};
  if (HasExtension(value, extension)) { return true; }
  printf("Invalid value for --%s: %s. File extension must be %s.\n", flagname,
         value.c_str(), extension.c_str());
  return false;
}

bool ValidateTxtFileMustExist(const char* flagname, const std::string& value) {
  if (!boost::filesystem::exists(value)) {
    printf("Invalid value for --%s: %s. File Does not exist.\n", flagname,
           value.c_str());
    return false;
  }

  std::string extension{".txt"};
  if (HasExtension(value, extension)) { return true; }
  printf("Invalid value for --%s: %s. File extension must be %s.\n", flagname,
         value.c_str(), extension.c_str());
  return false;
}

bool ValidatePlyFileMustExist(const char* flagname, const std::string& value) {
  if (!boost::filesystem::exists(value)) {
    printf("Invalid value for --%s: %s. File Does not exist.\n", flagname,
           value.c_str());
    return false;
  }

  std::string extension{".ply"};
  if (HasExtension(value, extension)) { return true; }
  printf("Invalid value for --%s: %s. File extension must be %s.\n", flagname,
         value.c_str(), extension.c_str());
  return false;
}

bool ValidateBagFileMustExist(const char* flagname, const std::string& value) {
  if (!boost::filesystem::exists(value)) {
    printf("Invalid value for --%s: %s. File Does not exist.\n", flagname,
           value.c_str());
    return false;
  }

  std::string extension{".bag"};
  if (HasExtension(value, extension)) { return true; }
  printf("Invalid value for --%s: %s. File extension must be %s.\n", flagname,
         value.c_str(), extension.c_str());
  return false;
}

bool ValidateMustBeJson(const char* flagname, const std::string& value) {
  std::string extension{".json"};
  if (HasExtension(value, extension)) { return true; }
  printf("Invalid value for --%s: %s. File extension must be %s.\n", flagname,
         value.c_str(), extension.c_str());
  return false;
}

}} // namespace beam::gflags
