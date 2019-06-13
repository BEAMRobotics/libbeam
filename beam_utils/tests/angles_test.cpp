#include "beam_utils/angles.hpp"
#include <catch2/catch.hpp>
#include <iostream>

constexpr float PI = 3.14159;

TEST_CASE("Input angles wrap to PI", "[Angles.hpp]") {

  REQUIRE(beam::wrapToPi(0.75*PI) == Approx(2.356).epsilon(0.01));
  REQUIRE(beam::wrapToPi(1.75*PI) == Approx(-0.785).epsilon(0.01));
  REQUIRE(beam::wrapToPi(2.75*PI) == Approx(2.356).epsilon(0.01));
  REQUIRE(beam::wrapToPi(3.75*PI) == Approx(-0.785).epsilon(0.01));
  REQUIRE(beam::wrapToPi(4.75*PI) == Approx(2.356).epsilon(0.01));

  REQUIRE(beam::wrapToPi(-0.75*PI) == Approx(-2.356).epsilon(0.01));
  REQUIRE(beam::wrapToPi(-1.75*PI) == Approx(0.785).epsilon(0.01));
  REQUIRE(beam::wrapToPi(-2.75*PI) == Approx(-2.356).epsilon(0.01));
  REQUIRE(beam::wrapToPi(-3.75*PI) == Approx(0.785).epsilon(0.01));
  REQUIRE(beam::wrapToPi(-4.75*PI) == Approx(-2.356).epsilon(0.01));

}

TEST_CASE("Degrees to radians", "[Angles.hpp]") {

  REQUIRE(beam::deg2rad(0) == Approx(0*PI).epsilon(0.01));
  REQUIRE(beam::deg2rad(90) == Approx(PI/2).epsilon(0.01));
  REQUIRE(beam::deg2rad(180) == Approx(PI).epsilon(0.01));
  REQUIRE(beam::deg2rad(270) == Approx(3*PI/2).epsilon(0.01));
  REQUIRE(beam::deg2rad(360) == Approx(2*PI).epsilon(0.01));

  REQUIRE(beam::deg2rad(-0) == Approx(-0*PI).epsilon(0.01));
  REQUIRE(beam::deg2rad(-90) == Approx(-PI/2).epsilon(0.01));
  REQUIRE(beam::deg2rad(-180) == Approx(-PI).epsilon(0.01));
  REQUIRE(beam::deg2rad(-270) == Approx(-3*PI/2).epsilon(0.01));
  REQUIRE(beam::deg2rad(-360) == Approx(-2*PI).epsilon(0.01));
}

TEST_CASE("Radians to degrees", "[Angles.hpp]") {

  REQUIRE(beam::rad2deg(0*PI) == Approx(0).epsilon(0.01));
  REQUIRE(beam::rad2deg(PI/2) == Approx(90).epsilon(0.01));
  REQUIRE(beam::rad2deg(PI) == Approx(180).epsilon(0.01));
  REQUIRE(beam::rad2deg(3*PI/2) == Approx(270).epsilon(0.01));
  REQUIRE(beam::rad2deg(2*PI) == Approx(360).epsilon(0.01));

  REQUIRE(beam::rad2deg(-0*PI) == Approx(0).epsilon(0.01));
  REQUIRE(beam::rad2deg(-PI/2) == Approx(-90).epsilon(0.01));
  REQUIRE(beam::rad2deg(-PI) == Approx(-180).epsilon(0.01));
  REQUIRE(beam::rad2deg(-3*PI/2) == Approx(-270).epsilon(0.01));
  REQUIRE(beam::rad2deg(-2*PI) == Approx(-360).epsilon(0.01));
}