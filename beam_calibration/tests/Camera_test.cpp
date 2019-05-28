#define CATCH_CONFIG_MAIN
#include "beam_calibration/CameraModel.h"
#include "beam_calibration/FisheyeCamera.h"
#include "beam_calibration/LadybugCamera.h"
#include "beam_calibration/PinholeCamera.h"
#include "beam_utils/math.hpp"
#include <catch2/catch.hpp>

/*TO DO
TEST_CASE("Test correct projection - radtan") {}

TEST_CASE("Test correct projection - equidistant") {}

TEST_CASE("Test correct projection - ladybug") {}
*/
TEST_CASE("Test factory method") {
  beam_calibration::CameraType type = beam_calibration::CameraType::NONE;
  beam::VecX intrinsics(4);
  intrinsics << 1, 2, 3, 4;
  beam::VecX distortion(5);
  distortion << 1, 2, 3, 4, 5;
  uint32_t image_width = 1000, image_height = 1000;
  std::string frame_id = "1", date = "now";
  // create camera model
  std::shared_ptr<beam_calibration::CameraModel> camera =
      beam_calibration::CameraModel::Create(type, intrinsics, distortion,
                                            image_height, image_width, frame_id,
                                            date);
  REQUIRE(!camera);
  type = beam_calibration::CameraType::PINHOLE;
  camera = beam_calibration::CameraModel::Create(
      type, intrinsics, distortion, image_height, image_width, frame_id, date);
  REQUIRE(camera->GetType() == beam_calibration::CameraType::PINHOLE);
  distortion.resize(4);
  type = beam_calibration::CameraType::FISHEYE;
  camera = beam_calibration::CameraModel::Create(
      type, intrinsics, distortion, image_height, image_width, frame_id, date);
  REQUIRE(camera->GetType() == beam_calibration::CameraType::FISHEYE);
}

TEST_CASE("Test exception throwing") {
  beam::Vec3 point(100, 40, 2000);
  beam::VecX intrinsics(4);
  intrinsics << 1, 2, 3, 4;
  beam::VecX distortion_radtan(5);
  distortion_radtan << 1, 2, 3, 4, 5;
  beam::VecX distortion_equid(4);
  distortion_equid << 1, 2, 3, 4;
  uint32_t image_width = 1000, image_height = 1000;
  std::string frame_id = "1", date = "now";
  beam_calibration::PinholeCamera radtan;
  beam::VecX invalid = beam::VecX::Zero(0);
  REQUIRE_THROWS(radtan.SetDistortionCoefficients(invalid));
  REQUIRE_THROWS(radtan.SetIntrinsics(invalid));
  REQUIRE_THROWS(radtan.ProjectPoint(point));
  REQUIRE_NOTHROW(radtan.SetIntrinsics(intrinsics));
  REQUIRE_NOTHROW(radtan.SetDistortionCoefficients(distortion_radtan));
  REQUIRE_NOTHROW(radtan.SetImageDims(image_width, image_height));
  REQUIRE_NOTHROW(radtan.ProjectPoint(point));

  beam_calibration::FisheyeCamera equid;
  REQUIRE_THROWS(equid.SetDistortionCoefficients(invalid));
  REQUIRE_THROWS(equid.SetIntrinsics(invalid));
  REQUIRE_THROWS(equid.ProjectPoint(point));
  REQUIRE_NOTHROW(equid.SetIntrinsics(intrinsics));
  REQUIRE_NOTHROW(equid.SetDistortionCoefficients(distortion_equid));
  REQUIRE_NOTHROW(equid.ProjectPoint(point));
}

TEST_CASE("Testing LoadJSON function") {
  // radtan
  std::string radtan_location = __FILE__;
  radtan_location.erase(radtan_location.end() - 21, radtan_location.end());
  radtan_location += "tests/test_data/F1.json";
  std::shared_ptr<beam_calibration::CameraModel> radtan =
      beam_calibration::CameraModel::LoadJSON(radtan_location);
  REQUIRE(radtan->GetWidth() == 2048);
  REQUIRE(radtan->GetHeight() == 1536);
  REQUIRE(radtan->GetFx() == 2338.485520924695);
  REQUIRE(radtan->GetFy() == 2333.0927287230647);
  REQUIRE(radtan->GetCx() == 1002.8381839138167);
  REQUIRE(radtan->GetCy() == 784.1498440053573);
  REQUIRE(radtan->GetDistortionCoefficients().size() == 5);
  REQUIRE(radtan->GetType() == beam_calibration::CameraType::PINHOLE);
  REQUIRE(radtan->GetIntrinsics().size() == 4);
  // load equidistant
  std::string equidistant_location = __FILE__;
  equidistant_location.erase(equidistant_location.end() - 21,
                             equidistant_location.end());
  equidistant_location += "tests/test_data/F2.json";
  std::shared_ptr<beam_calibration::CameraModel> equid =
      beam_calibration::CameraModel::LoadJSON(equidistant_location);
  REQUIRE(equid->GetWidth() == 2048);
  REQUIRE(equid->GetHeight() == 1536);
  REQUIRE(equid->GetFx() == 2338.485520924695);
  REQUIRE(equid->GetFy() == 2333.0927287230647);
  REQUIRE(equid->GetCx() == 1002.8381839138167);
  REQUIRE(equid->GetCy() == 784.1498440053573);
  REQUIRE(equid->GetDistortionCoefficients().size() == 4);
  REQUIRE(equid->GetType() == beam_calibration::CameraType::FISHEYE);
  REQUIRE(equid->GetIntrinsics().size() == 4);
}
