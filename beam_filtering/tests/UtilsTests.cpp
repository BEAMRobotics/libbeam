#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>
#include <nlohmann/json.hpp>

#include <beam_filtering/Utils.h>
#include <beam_utils/filesystem.h>

using namespace beam_filtering;

std::string GetFilePath() {
  std::string filename = "filter_params.json";
  std::string current_file = "UtilsTests.cpp";
  std::string filepath = __FILE__;
  filepath.erase(filepath.end() - current_file.length(), filepath.end());
  filepath += "test_data/" + filename;
  return filepath;
}

TEST_CASE("Test params read write with json") {
  std::string filepath = GetFilePath();
  nlohmann::json J;

  bool successful_read = beam::ReadJson(filepath, J);
  REQUIRE(successful_read);

  std::vector<FilterParamsType> params_vec = LoadFilterParamsVector(J["filters"]);

  int dror_counter = 0;
  int ror_counter = 0;
  int voxel_counter = 0;
  int cropbox_counter = 0;
  for (FilterParamsType filter_params : params_vec){
    if (filter_params.first == FilterType::CROPBOX){
      cropbox_counter++;
      std::vector<double> params = filter_params.second;
      REQUIRE(params.size() == 7);
      REQUIRE(params.at(0) == -0.3);
      REQUIRE(params.at(1) == -1);
      REQUIRE(params.at(2) == -1.5);
      REQUIRE(params.at(3) == 0.3);
      REQUIRE(params.at(4) == 1);
      REQUIRE(params.at(5) == 1.5);
      REQUIRE(params.at(6) == 0);
    } else if (filter_params.first == FilterType::DROR){
      dror_counter++;
      std::vector<double> params = filter_params.second;
      REQUIRE(params.size() == 4);
      REQUIRE(params.at(0) == 3);
      REQUIRE(params.at(1) == 0.04);
      REQUIRE(params.at(2) == 4);
      REQUIRE(params.at(3) == 0.02);
    } else if (filter_params.first == FilterType::ROR){
      ror_counter++;
      std::vector<double> params = filter_params.second;
      REQUIRE(params.size() == 2);
      REQUIRE(params.at(0) == 0.03);
      REQUIRE(params.at(1) == 5);
    } else if (filter_params.first == FilterType::VOXEL){
      voxel_counter++;
      std::vector<double> params = filter_params.second;
      REQUIRE(params.size() == 3);
      REQUIRE(params.at(0) == 0.051);
      REQUIRE(params.at(1) == 0.052);
      REQUIRE(params.at(2) == 0.053);
    } else {
      // throw error
      REQUIRE(false);
    }
  }

  REQUIRE(dror_counter == 1);
  REQUIRE(ror_counter == 1);
  REQUIRE(voxel_counter == 1);
  REQUIRE(cropbox_counter == 1);

}
