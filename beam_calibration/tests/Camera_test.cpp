#define CATCH_CONFIG_MAIN
#include "beam_calibration/CameraModel.h"
#include "beam_calibration/LadybugCamera.h"
#include "beam_calibration/PinholeCamera.h"
#include "beam_utils/math.hpp"
#include <catch2/catch.hpp>

TEST_CASE("Test projection and back project -- radtan") {
  std::string radtan_location = __FILE__;
  radtan_location.erase(radtan_location.end() - 21, radtan_location.end());
  radtan_location += "tests/test_data/F1.json";
  std::shared_ptr<beam_calibration::CameraModel> radtan =
      beam_calibration::CameraModel::LoadJSON(radtan_location);
  beam::Vec3 test_point(50, 50, 200);
  beam::Vec2 result_point = radtan->ProjectPoint(test_point);
  beam::Vec3 back_point = radtan->BackProject(result_point);

  std::stringstream bp, norm;
  bp << back_point;
  test_point.normalize();
  norm << test_point;
  INFO("Back projected point:");
  INFO(bp.str());
  INFO("Normalized input point:");
  INFO(norm.str());
  // require back projected point to be equal to test point normalized
  REQUIRE(beam::fltcmp(back_point[0], test_point[0], 0.01) == 0);
  REQUIRE(beam::fltcmp(back_point[1], test_point[1], 0.01) == 0);
  REQUIRE(beam::fltcmp(back_point[2], test_point[2], 0.01) == 0);
}

TEST_CASE("Test projection and back project -- equid") {
  std::string equid_location = __FILE__;
  equid_location.erase(equid_location.end() - 21, equid_location.end());
  equid_location += "tests/test_data/F2.json";
  std::shared_ptr<beam_calibration::CameraModel> equid =
      beam_calibration::CameraModel::LoadJSON(equid_location);
  beam::Vec3 test_point(123, 252, 531);
  beam::Vec2 result_point = equid->ProjectPoint(test_point);
  beam::Vec3 back_point = equid->BackProject(result_point);

  std::stringstream bp, norm;
  bp << back_point;
  test_point.normalize();
  norm << test_point;
  INFO("Back projected point:");
  INFO(bp.str());
  INFO("Normalized input point:");
  INFO(norm.str());
  // require back projected point to be equal to test point normalized
  REQUIRE(beam::fltcmp(back_point[0], test_point[0], 0.01) == 0);
  REQUIRE(beam::fltcmp(back_point[1], test_point[1], 0.01) == 0);
  REQUIRE(beam::fltcmp(back_point[2], test_point[2], 0.01) == 0);
}

TEST_CASE("Test distortion and undistortion") {
  std::string radtan_location = __FILE__;
  radtan_location.erase(radtan_location.end() - 21, radtan_location.end());
  radtan_location += "tests/test_data/F1.json";
  std::shared_ptr<beam_calibration::CameraModel> radtan =
      beam_calibration::CameraModel::LoadJSON(radtan_location);
  std::string equid_location = __FILE__;
  equid_location.erase(equid_location.end() - 21, equid_location.end());
  equid_location += "tests/test_data/F2.json";
  std::shared_ptr<beam_calibration::CameraModel> equid =
      beam_calibration::CameraModel::LoadJSON(equid_location);

  std::stringstream og1, og2, d1, und1, d2, und2;
  beam::Vec2 original(30, 100);
  beam::Vec2 distorted = equid->DistortPoint(original);
  beam::Vec2 undistorted = equid->UndistortPoint(distorted);
  REQUIRE(beam::fltcmp(original[0], undistorted[0]) == 0);
  REQUIRE(beam::fltcmp(original[1], undistorted[1]) == 0);
  og1 << original;
  d1 << distorted;
  und1 << undistorted;
  INFO(og1.str());
  INFO(d1.str());
  INFO(und1.str());

  beam::Vec2 original2(10, 10);
  beam::Vec2 distorted2 = radtan->DistortPoint(original2);
  beam::Vec2 undistorted2 = radtan->UndistortPoint(distorted2);
  og2 << original2;
  d2 << distorted2;
  und2 << undistorted2;
  INFO(og2.str());
  INFO(d2.str());
  INFO(und2.str());
  REQUIRE(beam::fltcmp(original2[0], undistorted2[0]) == 0);
  REQUIRE(beam::fltcmp(original2[1], undistorted2[1]) == 0);
}

TEST_CASE("Test factory method") {
  beam_calibration::CameraType type = beam_calibration::CameraType::PINHOLE;
  beam_calibration::DistortionType dist_type =
      beam_calibration::DistortionType::RADTAN;
  beam::VecX intrinsics(4);
  intrinsics << 1, 2, 3, 4;
  beam::VecX distortion(5);
  distortion << 1, 2, 3, 4, 5;
  uint32_t image_width = 1000, image_height = 1000;
  std::string frame_id = "1", date = "now";
  // create camera model
  std::shared_ptr<beam_calibration::CameraModel> camera =
      beam_calibration::CameraModel::Create(type, dist_type, intrinsics,
                                            distortion, image_height,
                                            image_width, frame_id, date);
  REQUIRE(camera);
  REQUIRE(camera->GetType() == beam_calibration::CameraType::PINHOLE);
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
  beam_calibration::PinholeCamera pinhole;
  beam::VecX invalid = beam::VecX::Zero(0);
  beam_calibration::DistortionType dist_type =
      beam_calibration::DistortionType::RADTAN;

  REQUIRE_THROWS(pinhole.SetDistortionCoefficients(invalid));
  REQUIRE_THROWS(pinhole.SetIntrinsics(invalid));
  REQUIRE_THROWS(pinhole.ProjectPoint(point));

  REQUIRE_NOTHROW(pinhole.SetDistortionType(dist_type));
  REQUIRE_NOTHROW(pinhole.SetIntrinsics(intrinsics));
  REQUIRE_NOTHROW(pinhole.SetDistortionCoefficients(distortion_radtan));
  REQUIRE_NOTHROW(pinhole.SetImageDims(image_width, image_height));
  REQUIRE_NOTHROW(pinhole.ProjectPoint(point));
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
  REQUIRE(radtan->GetDistortionType() ==
          beam_calibration::DistortionType::RADTAN);
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
  REQUIRE(equid->GetFx() == 783.44463219576687);
  REQUIRE(equid->GetFy() == 783.68479107567089);
  REQUIRE(equid->GetCx() == 996.34300258081578);
  REQUIRE(equid->GetCy() == 815.47561902246832);
  REQUIRE(equid->GetDistortionCoefficients().size() == 4);
  REQUIRE(equid->GetType() == beam_calibration::CameraType::PINHOLE);
  REQUIRE(equid->GetDistortionType() ==
          beam_calibration::DistortionType::EQUIDISTANT);
  REQUIRE(equid->GetIntrinsics().size() == 4);
}
