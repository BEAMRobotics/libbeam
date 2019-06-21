#include "beam_utils/math.hpp"
#include <catch2/catch.hpp>
#include <iostream>

constexpr float PI = 3.14159;

TEST_CASE("Float comparator", "[Math.hpp]") {
  float f1 = 2.2345642;
  float f2 = 2.2345652;
  REQUIRE(beam::fltcmp(f1, f2, 0.000001) == 0);
  REQUIRE(beam::fltcmp(f1, f2, 0.0000001) == -1);
  REQUIRE(beam::fltcmp(f2, f1, 0.0000001) == 1);
}

TEST_CASE("Median", "[Math.hpp]") {
  std::vector<double> vec;
  for (int i = 1; i < 10; i++) { vec.push_back(i); }
  REQUIRE(beam::median(vec) == 5);
  vec.push_back(2);
  REQUIRE(beam::median(vec) == 4.5);
}

TEST_CASE("Distance", "[Math.hpp]") {
  beam::Vec3 point1(0, 0, 0);
  beam::Vec3 point2(100, 20, 30);
  beam::Vec3 point3(100, 10, 20);
  REQUIRE(beam::distance(point1, point2) == Approx(106.301).epsilon(0.01));
  REQUIRE(beam::distance(point1, point3) == Approx(102.47).epsilon(0.01));
  REQUIRE(beam::distance(point3, point2) == Approx(14.1421).epsilon(0.01));
}
