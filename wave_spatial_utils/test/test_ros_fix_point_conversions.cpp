/* Copyright (c) 2017-2018, Waterloo Autonomous Vehicles Laboratory (WAVELab),
 * Waterloo Intelligent Systems Engineering Lab (WISELab),
 * University of Waterloo.
 *
 * Refer to the accompanying LICENSE file for license information.
 *
 * ############################################################################
 ******************************************************************************
 |                                                                            |
 |                         /\/\__/\_/\      /\_/\__/\/\                       |
 |                         \          \____/          /                       |
 |                          '----________________----'                        |
 |                              /                \                            |
 |                            O/_____/_______/____\O                          |
 |                            /____________________\                          |
 |                           /    (#UNIVERSITY#)    \                         |
 |                           |[**](#OFWATERLOO#)[**]|                         |
 |                           \______________________/                         |
 |                            |_""__|_,----,_|__""_|                          |
 |                            ! !                ! !                          |
 |                            '-'                '-'                          |
 |       __    _   _  _____  ___  __  _  ___  _    _  ___  ___   ____  ____   |
 |      /  \  | | | ||_   _|/ _ \|  \| |/ _ \| \  / |/ _ \/ _ \ /     |       |
 |     / /\ \ | |_| |  | |  ||_||| |\  |||_|||  \/  |||_||||_|| \===\ |====   |
 |    /_/  \_\|_____|  |_|  \___/|_| \_|\___/|_|\/|_|\___/\___/ ____/ |____   |
 |                                                                            |
 ******************************************************************************
 * ############################################################################
 *
 * File: test_ros_fix_point_conversions.cpp
 * Desc: Tests for LLH <-> ENU point conversions
 * Auth: Michael Smart <michael.smart@uwaterloo.ca>
 *
 * ############################################################################
*/

#include <gtest/gtest.h>
#include "wave_spatial_utils/world_frame_conversions.hpp"

namespace wave_spatial_utils {

// These functions just wrap the llhPointFromENU and enuPointFromLLH functions
// So just check that the conversions loop properly for a single square of
// points with the datum at A:
//
//      A -- B
//      |    |
//      C -- D

TEST(ROSFixAndPointConversionsTest, LoopTest) {
    // Pick a datum
    sensor_msgs::NavSatFix datum;
    datum.latitude = 45;
    datum.longitude = 90;
    datum.altitude = -100;

    // Scale at 150m square
    geometry_msgs::Point ground_truth_A, ground_truth_B, ground_truth_C,
      ground_truth_D;
    ground_truth_A.x = 0;
    ground_truth_A.y = 0;
    ground_truth_A.z = 0;
    ground_truth_B.x = 150.0;
    ground_truth_B.y = 0;
    ground_truth_B.z = 0;
    ground_truth_C.x = 0;
    ground_truth_C.y = -150.0;
    ground_truth_C.z = 0;
    ground_truth_D.x = 150.0;
    ground_truth_D.y = -150.0;
    ground_truth_D.z = 0;

    // Convert points to navsat
    sensor_msgs::NavSatFix navsat_A, navsat_B, navsat_C, navsat_D;
    point_to_fix(ground_truth_A, datum, &navsat_A);
    point_to_fix(ground_truth_B, datum, &navsat_B);
    point_to_fix(ground_truth_C, datum, &navsat_C);
    point_to_fix(ground_truth_D, datum, &navsat_D);

    // Check relative heights
    EXPECT_LT(navsat_A.altitude, navsat_B.altitude);
    EXPECT_LT(navsat_A.altitude, navsat_C.altitude);
    EXPECT_LT(navsat_B.altitude, navsat_D.altitude);
    EXPECT_LT(navsat_C.altitude, navsat_D.altitude);
    // Check relative latitudes
    EXPECT_LT(navsat_C.latitude, navsat_A.latitude);
    EXPECT_LT(navsat_C.latitude, navsat_B.latitude);
    EXPECT_LT(navsat_D.latitude, navsat_A.latitude);
    EXPECT_LT(navsat_D.latitude, navsat_B.latitude);
    // Check relative longitudes
    EXPECT_LT(navsat_A.longitude, navsat_B.longitude);
    EXPECT_LT(navsat_A.longitude, navsat_D.longitude);
    EXPECT_LT(navsat_C.longitude, navsat_B.longitude);
    EXPECT_LT(navsat_C.longitude, navsat_D.longitude);

    // Convert back
    geometry_msgs::Point point_A, point_B, point_C, point_D;
    fix_to_point(navsat_A, datum, &point_A);
    fix_to_point(navsat_B, datum, &point_B);
    fix_to_point(navsat_C, datum, &point_C);
    fix_to_point(navsat_D, datum, &point_D);

    // Check matches against ground truth values
    EXPECT_NEAR(point_A.x, ground_truth_A.x, 1e-6);
    EXPECT_NEAR(point_A.y, ground_truth_A.y, 1e-6);
    EXPECT_NEAR(point_A.z, ground_truth_A.z, 1e-6);

    EXPECT_NEAR(point_B.x, ground_truth_B.x, 1e-6);
    EXPECT_NEAR(point_B.y, ground_truth_B.y, 1e-6);
    EXPECT_NEAR(point_B.z, ground_truth_B.z, 1e-6);

    EXPECT_NEAR(point_C.x, ground_truth_C.x, 1e-6);
    EXPECT_NEAR(point_C.y, ground_truth_C.y, 1e-6);
    EXPECT_NEAR(point_C.z, ground_truth_C.z, 1e-6);

    EXPECT_NEAR(point_D.x, ground_truth_D.x, 1e-6);
    EXPECT_NEAR(point_D.y, ground_truth_D.y, 1e-6);
    EXPECT_NEAR(point_D.z, ground_truth_D.z, 1e-6);
}

}  // namespace wave_spatial_utils
