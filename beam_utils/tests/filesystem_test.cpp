#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>
#include <catch2/catch.hpp>

#include <beam_utils/filesystem.h>

namespace fs = boost::filesystem;

void CreateFile(const std::string& name, std::vector<std::string>& paths) {
  std::ofstream outfile(name);
  outfile.close();
  paths.push_back(name);
}

bool AreListsEqual(std::vector<std::string>& paths1,
                   std::vector<std::string>& paths2) {
  bool lists_equal = true;

  if (paths1.size() != paths2.size()) { lists_equal = false; }

  for (std::string path : paths1) {
    // check for current file path in second list
    if (std::find(paths2.begin(), paths2.end(), path) == paths2.end()) {
      lists_equal = false;
      break;
    }
  }

  if (!lists_equal) {
    std::cout << "Lists are not equal.\nList 1:\n";
    for (std::string path : paths1) { std::cout << path << "\n"; }
    std::cout << "List 2:\n";
    for (std::string path : paths2) { std::cout << path << "\n"; }
  }
  return lists_equal;
}

TEST_CASE("GetFiles tests", "[filesystem.hpp]") {
  // create directories
  std::string save_path =
      fs::temp_directory_path().string() + "/libbeam_filesystem_test/";
  if (!fs::is_directory(save_path)) { fs::create_directory(save_path); }

  fs::create_directory(save_path + "subdir1/");
  fs::create_directory(save_path + "subdir2/");

  // create files
  std::vector<std::string> txt_files;
  std::vector<std::string> other_files;
  CreateFile(save_path + "test1.txt", txt_files);
  CreateFile(save_path + "test2.txt", txt_files);
  std::vector<std::string> root_files{save_path + "test1.txt",
                                      save_path + "test2.txt"};
  CreateFile(save_path + "subdir1/test3.txt", txt_files);
  CreateFile(save_path + "subdir1/test4.txt", txt_files);
  CreateFile(save_path + "subdir2/test5.txt", txt_files);
  CreateFile(save_path + "subdir2/test6.txt", txt_files);
  CreateFile(save_path + "subdir2/test7.csv", other_files);
  std::vector<std::string> all_files = txt_files;
  all_files.push_back(other_files[0]);

  std::vector<std::string> text_files_found_nonrecursive =
      beam::GetFiles(save_path, ".txt", false);
  REQUIRE(AreListsEqual(text_files_found_nonrecursive, root_files));

  std::vector<std::string> all_files_found_nonrecursive =
      beam::GetFiles(save_path, "", false);
  REQUIRE(AreListsEqual(all_files_found_nonrecursive, root_files));

  std::vector<std::string> text_files_found_recursive =
      beam::GetFiles(save_path, ".txt", true);
  REQUIRE(AreListsEqual(text_files_found_recursive, txt_files));

  std::vector<std::string> all_files_found_recursive =
      beam::GetFiles(save_path, "", true);
  REQUIRE(AreListsEqual(all_files_found_recursive, all_files));

  // remove files and check they are gone
  fs::remove_all(save_path + "subdir1");
  std::vector<std::string> should_be_empty =
      beam::GetFiles(save_path + "subdir1", "", true);
  REQUIRE(should_be_empty.empty());

  fs::remove_all(save_path);
  should_be_empty = beam::GetFiles(save_path, "", true);
  REQUIRE(should_be_empty.empty());
}

TEST_CASE("LibbeamRoot tests", "[filesystem.hpp]") {
  std::string root_dir = beam::LibbeamRoot();

  // get path to this file and make sure it exists and is equal to __FILE__
  std::string this_file_true_location = __FILE__;
  std::string this_file_expected_location =
      root_dir + "beam_utils/tests/filesystem_test.cpp";
  REQUIRE(fs::exists(this_file_expected_location));
  REQUIRE(this_file_true_location == this_file_expected_location);
}

TEST_CASE("StringToNumericValues", "[filesystem.hpp]") {
  std::string deliminator_1{","};
  std::string deliminator_2{" "};
  std::string input_string{"0 3.14159265359 -3.14159265359 10"};
  std::vector<double> values;
  REQUIRE(beam::StringToNumericValues(deliminator_1, input_string, values) ==
          false);
  REQUIRE(beam::StringToNumericValues(deliminator_2, input_string, values));

  std::vector<double> expected_values{0, 3.14159265359, -3.14159265359, 10};
  REQUIRE(expected_values.at(0) == values.at(0));
  REQUIRE(expected_values.at(1) == values.at(1));
  REQUIRE(expected_values.at(2) == values.at(2));
  REQUIRE(expected_values.at(3) == values.at(3));
}