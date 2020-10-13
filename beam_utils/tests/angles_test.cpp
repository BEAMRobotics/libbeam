#include "beam_utils/angles.hpp"
#include <catch2/catch.hpp>
#include <iostream>

constexpr float PI = M_PI;

TEST_CASE("Input angles wrap to PI", "[Angles.hpp]") {
  // test first quadrant
  REQUIRE(beam::WrapToPi((0.25+0)*PI) == Approx(0.25*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((0.25+2)*PI) == Approx(0.25*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((0.25+4)*PI) == Approx(0.25*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((0.25-2)*PI) == Approx(0.25*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((0.25-4)*PI) == Approx(0.25*PI).epsilon(0.0001));

  // test second quadrant
  REQUIRE(beam::WrapToPi((0.75+0)*PI) == Approx(0.75*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((0.75+2)*PI) == Approx(0.75*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((0.75+4)*PI) == Approx(0.75*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((0.75-2)*PI) == Approx(0.75*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((0.75-4)*PI) == Approx(0.75*PI).epsilon(0.0001));

  // third quadrant
  REQUIRE(beam::WrapToPi((-0.75+0)*PI) == Approx(-0.75*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((-0.75+2)*PI) == Approx(-0.75*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((-0.75+4)*PI) == Approx(-0.75*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((-0.75-2)*PI) == Approx(-0.75*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((-0.75-4)*PI) == Approx(-0.75*PI).epsilon(0.0001));

  // fourth quadrant
  REQUIRE(beam::WrapToPi((-0.25+0)*PI) == Approx(-0.25*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((-0.25+2)*PI) == Approx(-0.25*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((-0.25+4)*PI) == Approx(-0.25*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((-0.25-2)*PI) == Approx(-0.25*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((-0.25-4)*PI) == Approx(-0.25*PI).epsilon(0.0001));
}

TEST_CASE("Input angles wrap to 2PI", "[Angles.hpp]") {
  // test first quadrant
  REQUIRE(beam::WrapToPi((0.25+0)*PI) == Approx(0.25*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((0.25+2)*PI) == Approx(0.25*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((0.25+4)*PI) == Approx(0.25*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((0.25-2)*PI) == Approx(0.25*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((0.25-4)*PI) == Approx(0.25*PI).epsilon(0.0001));

  // test second quadrant
  REQUIRE(beam::WrapToPi((0.75+0)*PI) == Approx(0.75*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((0.75+2)*PI) == Approx(0.75*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((0.75+4)*PI) == Approx(0.75*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((0.75-2)*PI) == Approx(0.75*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((0.75-4)*PI) == Approx(0.75*PI).epsilon(0.0001));

  // third quadrant
  REQUIRE(beam::WrapToPi((1.25+0)*PI) == Approx(-0.75*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((1.25+2)*PI) == Approx(-0.75*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((1.25+4)*PI) == Approx(-0.75*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((1.25-2)*PI) == Approx(-0.75*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((1.25-4)*PI) == Approx(-0.75*PI).epsilon(0.0001));

  // fourth quadrant
  REQUIRE(beam::WrapToPi((1.75+0)*PI) == Approx(-0.25*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((1.75+2)*PI) == Approx(-0.25*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((1.75+4)*PI) == Approx(-0.25*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((1.75-2)*PI) == Approx(-0.25*PI).epsilon(0.0001));
  REQUIRE(beam::WrapToPi((1.75-4)*PI) == Approx(-0.25*PI).epsilon(0.0001));
}

TEST_CASE("Test getting error between two angles", "[Angles.hpp]") {
  // test quad 1 + quad 4 cases
  REQUIRE(beam::GetSmallestAngleErrorRad((0.25+0)*PI, (-0.25+0)*PI) == Approx(0.5*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((-0.25+0)*PI, (0.25+0)*PI) == Approx(0.5*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((-0.1+0)*PI, (0.1+0)*PI) == Approx(0.2*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((0.1+0)*PI, (-0.1+0)*PI) == Approx(0.2*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((0.25+0)*PI, (-0.25+2)*PI) == Approx(0.5*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((-0.25+2)*PI, (0.25+2)*PI) == Approx(0.5*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((-0.1-2)*PI, (0.1+0)*PI) == Approx(0.2*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((0.1-2)*PI, (-0.1+0)*PI) == Approx(0.2*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((-0.1-2)*PI, (0.1-2)*PI) == Approx(0.2*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((0.1-2)*PI, (-0.1+2)*PI) == Approx(0.2*PI).epsilon(0.0001));

  // test first and second quad
  REQUIRE(beam::GetSmallestAngleErrorRad((0.25+0)*PI, (0.75+0)*PI) == Approx(0.5*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((0.25+0)*PI, (0.75-2)*PI) == Approx(0.5*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((0.25+0)*PI, (0.75+2)*PI) == Approx(0.5*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((0.25-2)*PI, (0.75-0)*PI) == Approx(0.5*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((0.25+2)*PI, (0.75+0)*PI) == Approx(0.5*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((0.25-2)*PI, (0.75-2)*PI) == Approx(0.5*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((0.25+2)*PI, (0.75+2)*PI) == Approx(0.5*PI).epsilon(0.0001));

  // test second and third quad
  REQUIRE(beam::GetSmallestAngleErrorRad((1.25+0)*PI, (0.75+0)*PI) == Approx(0.5*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((1.25+0)*PI, (0.75-2)*PI) == Approx(0.5*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((1.25+0)*PI, (0.75+2)*PI) == Approx(0.5*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((1.25-2)*PI, (0.75-0)*PI) == Approx(0.5*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((1.25+2)*PI, (0.75+0)*PI) == Approx(0.5*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((1.25-2)*PI, (0.75-2)*PI) == Approx(0.5*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((1.25+2)*PI, (0.75+2)*PI) == Approx(0.5*PI).epsilon(0.0001));

  // test third and fourth quad
  REQUIRE(beam::GetSmallestAngleErrorRad((1.25+0)*PI, (1.75+0)*PI) == Approx(0.5*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((1.25+0)*PI, (1.75-2)*PI) == Approx(0.5*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((1.25+0)*PI, (1.75+2)*PI) == Approx(0.5*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((1.25-2)*PI, (1.75-0)*PI) == Approx(0.5*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((1.25+2)*PI, (1.75+0)*PI) == Approx(0.5*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((1.25-2)*PI, (1.75-2)*PI) == Approx(0.5*PI).epsilon(0.0001));
  REQUIRE(beam::GetSmallestAngleErrorRad((1.25+2)*PI, (1.75+2)*PI) == Approx(0.5*PI).epsilon(0.0001));


}

TEST_CASE("Degrees to radians", "[Angles.hpp]") {
  REQUIRE(beam::Deg2Rad(0) == Approx(0*PI).epsilon(0.0001));
  REQUIRE(beam::Deg2Rad(90) == Approx(PI/2).epsilon(0.0001));
  REQUIRE(beam::Deg2Rad(180) == Approx(PI).epsilon(0.0001));
  REQUIRE(beam::Deg2Rad(270) == Approx(3*PI/2).epsilon(0.0001));
  REQUIRE(beam::Deg2Rad(360) == Approx(2*PI).epsilon(0.0001));

  REQUIRE(beam::Deg2Rad(-0) == Approx(-0*PI).epsilon(0.0001));
  REQUIRE(beam::Deg2Rad(-90) == Approx(-PI/2).epsilon(0.0001));
  REQUIRE(beam::Deg2Rad(-180) == Approx(-PI).epsilon(0.0001));
  REQUIRE(beam::Deg2Rad(-270) == Approx(-3*PI/2).epsilon(0.0001));
  REQUIRE(beam::Deg2Rad(-360) == Approx(-2*PI).epsilon(0.0001));
}

TEST_CASE("Radians to degrees", "[Angles.hpp]") {
  REQUIRE(beam::Rad2Deg(0*PI) == Approx(0).epsilon(0.0001));
  REQUIRE(beam::Rad2Deg(PI/2) == Approx(90).epsilon(0.0001));
  REQUIRE(beam::Rad2Deg(PI) == Approx(180).epsilon(0.0001));
  REQUIRE(beam::Rad2Deg(3*PI/2) == Approx(270).epsilon(0.0001));
  REQUIRE(beam::Rad2Deg(2*PI) == Approx(360).epsilon(0.0001));

  REQUIRE(beam::Rad2Deg(-0*PI) == Approx(0).epsilon(0.0001));
  REQUIRE(beam::Rad2Deg(-PI/2) == Approx(-90).epsilon(0.0001));
  REQUIRE(beam::Rad2Deg(-PI) == Approx(-180).epsilon(0.0001));
  REQUIRE(beam::Rad2Deg(-3*PI/2) == Approx(-270).epsilon(0.0001));
  REQUIRE(beam::Rad2Deg(-2*PI) == Approx(-360).epsilon(0.0001));
}
