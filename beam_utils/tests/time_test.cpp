#include "beam_utils/time.h"
#include <catch2/catch.hpp>
#include <iostream>

TEST_CASE("ros and chrono conversions", "[Time.h]") {
  uint64_t seconds = 987;
  uint64_t nanoseconds = 654321012;
  ros::Time ros_time_true(seconds, nanoseconds);
  std::chrono::seconds seconds_chrono(seconds);
  std::chrono::nanoseconds nanoseconds_chrono(nanoseconds);
  beam::TimePoint time_point_true(seconds_chrono + nanoseconds_chrono);
  beam::TimePoint time_point_returned = beam::RosTimeToChrono(ros_time_true);
  ros::Time ros_time_returned = beam::ChronoToRosTime(time_point_true);
  REQUIRE(ros_time_returned.sec == seconds);
  REQUIRE(ros_time_returned.nsec == nanoseconds);
  REQUIRE(ros_time_returned == ros_time_true);
  REQUIRE(time_point_true.time_since_epoch().count() ==
          time_point_returned.time_since_epoch().count());
}
