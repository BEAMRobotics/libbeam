#define CATCH_CONFIG_MAIN
#include "beam_calibration/CameraModel.h"
#include "beam_calibration/DoubleSphere.h"
#include "beam_calibration/KannalaBrandt.h"
#include "beam_calibration/Ladybug.h"
#include "beam_calibration/Radtan.h"
#include "beam_utils/math.hpp"
#include <catch2/catch.hpp>

using namespace std::experimental;

TEST_CASE("Test projection and back project -- radtan") {
  std::string radtan_location = __FILE__;
  radtan_location.erase(radtan_location.end() - 21, radtan_location.end());
  radtan_location += "tests/test_data/Radtan_test.json";

  std::unique_ptr<beam_calibration::CameraModel> radtan =
      std::make_unique<beam_calibration::Radtan>(radtan_location);

  Eigen::Vector3d test_point(50, 50, 200);
  optional<Eigen::Vector2i> result_point = radtan->ProjectPoint(test_point);
  optional<Eigen::Vector3d> back_point =
      radtan->BackProject(result_point.value());

  std::stringstream bp, norm;
  bp << back_point.value();
  test_point.normalize();
  norm << test_point;
  INFO("Back projected point:");
  INFO(bp.str());
  INFO("Normalized input point:");
  INFO(norm.str());
  // require back projected point to be equal to test point normalized
  REQUIRE(beam::fltcmp(back_point.value()[0], test_point[0], 0.01) == 0);
  REQUIRE(beam::fltcmp(back_point.value()[1], test_point[1], 0.01) == 0);
  REQUIRE(beam::fltcmp(back_point.value()[2], test_point[2], 0.01) == 0);
}

TEST_CASE("Test projection and back project -- kannala brandt") {
  std::string kb_location = __FILE__;
  kb_location.erase(kb_location.end() - 21, kb_location.end());
  kb_location += "tests/test_data/KB_test.json";

  std::unique_ptr<beam_calibration::CameraModel> kb =
      std::make_unique<beam_calibration::KannalaBrandt>(kb_location);
  Eigen::Vector3d test_point(5, 10, 15);
  optional<Eigen::Vector2i> result_point = kb->ProjectPoint(test_point);
  optional<Eigen::Vector3d> back_point = kb->BackProject(result_point.value());

  std::stringstream bp, norm;
  bp << back_point.value();
  test_point.normalize();
  norm << test_point;
  INFO("Back projected point:");
  INFO(bp.str());
  INFO("Normalized input point:");
  INFO(norm.str());
  // require back projected point to be equal to test point normalized
  REQUIRE(beam::fltcmp(back_point.value()[0], test_point[0], 0.01) == 0);
  REQUIRE(beam::fltcmp(back_point.value()[1], test_point[1], 0.01) == 0);
  REQUIRE(beam::fltcmp(back_point.value()[2], test_point[2], 0.01) == 0);
}

TEST_CASE("Test projection and back project -- double sphere") {
  std::string db_location = __FILE__;
  db_location.erase(db_location.end() - 21, db_location.end());
  db_location += "tests/test_data/DS_test.json";

  std::unique_ptr<beam_calibration::CameraModel> db =
      std::make_unique<beam_calibration::DoubleSphere>(db_location);
  Eigen::Vector3d test_point(50, 50, 200);
  optional<Eigen::Vector2i> result_point = db->ProjectPoint(test_point);
  optional<Eigen::Vector3d> back_point = db->BackProject(result_point.value());

  std::stringstream bp, norm;
  bp << back_point.value();
  test_point.normalize();
  norm << test_point;
  INFO("Back projected point:");
  INFO(bp.str());
  INFO("Normalized input point:");
  INFO(norm.str());
  // require back projected point to be equal to test point normalized
  REQUIRE(beam::fltcmp(back_point.value()[0], test_point[0], 0.01) == 0);
  REQUIRE(beam::fltcmp(back_point.value()[1], test_point[1], 0.01) == 0);
  REQUIRE(beam::fltcmp(back_point.value()[2], test_point[2], 0.01) == 0);
}
/*
TEST_CASE("Test projection and back project -- ladybug") {
  std::string lb_location = __FILE__;
  lb_location.erase(lb_location.end() - 21, lb_location.end());
  lb_location += "tests/test_data/ladybug.conf";

  std::unique_ptr<beam_calibration::CameraModel> lb =
      std::make_unique<beam_calibration::Ladybug>(lb_location);

  Eigen::Vector3d test_point(50, 50, 200);
  optional<Eigen::Vector2i> result_point = lb->ProjectPoint(test_point);
  optional<Eigen::Vector3d> back_point = lb->BackProject(result_point.value());

  std::stringstream bp, norm;
  bp << back_point.value();
  test_point.normalize();
  norm << test_point;
  INFO("Back projected point:");
  INFO(bp.str());
  INFO("Normalized input point:");
  INFO(norm.str());
  // require back projected point to be equal to test point normalized
  REQUIRE(beam::fltcmp(back_point.value()[0], test_point[0], 0.01) == 0);
  REQUIRE(beam::fltcmp(back_point.value()[1], test_point[1], 0.01) == 0);
  REQUIRE(beam::fltcmp(back_point.value()[2], test_point[2], 0.01) == 0);
}*/

TEST_CASE("Testing LoadJSON function") {}
