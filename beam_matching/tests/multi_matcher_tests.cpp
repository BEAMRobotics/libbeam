#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>
#include <pcl/io/pcd_io.h>

#include <beam_matching/IcpMatcher.h>
#include <beam_matching/MultiMatcher.h>

namespace beam_matching {

PointCloudPtr cld_;
MultiMatcher<IcpMatcher, IcpMatcherParams> matcher_;

void SetUp() {
  std::string test_path = __FILE__;
  std::string current_file = "multi_matcher_tests.cpp";
  test_path.erase(test_path.end() - current_file.size(), test_path.end());
  std::string scan_path = test_path + "data/testscan.pcd";

  cld_ = std::make_shared<PointCloud>();
  pcl::io::loadPCDFile(scan_path, *(cld_));
}

TEST_CASE("Test simultaneous matching") {
  SetUp();
  PointCloudPtr dupes[9];
  for (int i = 0; i < 9; i++) {
    dupes[i] = std::make_shared<PointCloud>();
    *(dupes[i]) = *(cld_);
  }
  for (int i = 0; i < 8; i++) {
    matcher_.Insert(i, dupes[i], dupes[i + 1]);
  }
  while (!matcher_.Done()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

} // namespace beam_matching
