#include <beam_utils/gflags.h>
#include <boost/filesystem.hpp>

namespace beam { namespace gflags {

bool IsExtension(const std::string& input, const std::string& should_be) {
  std::string file_extension = input;
  file_extension.erase(file_extension.end() - input.length(),
                       file_extension.end() - should_be.length());
  if (file_extension != should_be) { return false; }
  return true;
}

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
  if (!boost::filesystem::exists(value)) {
    printf("Invalid value for --%s: %s. Directory does not exist.\n", flagname,
           value.c_str());
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
  if (IsExtension(value, extension)) { return true; }
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
  if (IsExtension(value, extension)) { return true; }
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
  if (IsExtension(value, extension)) { return true; }
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
  if (IsExtension(value, extension)) { return true; }
  printf("Invalid value for --%s: %s. File extension must be %s.\n", flagname,
         value.c_str(), extension.c_str());
  return false;
}

bool ValidateMustBeJson(const char* flagname, const std::string& value) {
  std::string extension{".json"};
  if (IsExtension(value, extension)) { return true; }
  printf("Invalid value for --%s: %s. File extension must be %s.\n", flagname,
         value.c_str(), extension.c_str());
  return false;
}

}} // namespace beam::gflags
