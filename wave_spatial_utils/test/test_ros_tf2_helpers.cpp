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
 * File: test_ros_tf_helpers.cpp
 * Desc: Tests for LLH <-> ENU point conversions
 * Auth: Michael Smart <michael.smart@uwaterloo.ca>
 *
 * ############################################################################
*/

#include <gtest/gtest.h>
#include "wave_spatial_utils/world_frame_conversions.hpp"

namespace wave_spatial_utils {

// These functions just wrap the other functions in the
// world_frame_conversions header to produce ros tf2::Transforms
// which have more detailed tests covering their conversion values
//
// Therefore, only need to test that the interfaces work
//
// Use latitude = 0, longitude = 90, altitude = 0 for all
//
// At (0,0,0):
//      +X <-> +U
//      +Y <-> +E
//      +Z <-> +N
//
// R_ENU_ECEF is near
//
// X = [0, 0, 1]^T
// Y = [1, 0, 0]^T
// Z = [0, 1, 0]^T
//
// R = [X|Y|Z]
//
// and the origin of ECEF is located at approximately (WGS84):
// [0, 0, -6,378.1370km = -6.3781370e6m]_enu
//
// the origin of ENU is located at:
// [-6.3781370e6m, 0, 0]_ecef

class rosTf2HelpersTest : public ::testing::Test {
 protected:
    double equatorial_radius = 6.3781370e6;
    double half_power = 1 / sqrt(2.0);

    void checkTfEnuEcefForLat0Lon0(const tf2::Transform &tr) {
        // At (l,l,a) = (0,0,0) these results should be very close
        EXPECT_NEAR(0, tr.getBasis().getRow(0).getX(), 1e-12);
        EXPECT_NEAR(1, tr.getBasis().getRow(0).getY(), 1e-12);
        EXPECT_NEAR(0, tr.getBasis().getRow(0).getZ(), 1e-12);
        EXPECT_NEAR(0, tr.getBasis().getRow(1).getX(), 1e-12);
        EXPECT_NEAR(0, tr.getBasis().getRow(1).getY(), 1e-12);
        EXPECT_NEAR(1, tr.getBasis().getRow(1).getZ(), 1e-12);
        EXPECT_NEAR(1, tr.getBasis().getRow(2).getX(), 1e-12);
        EXPECT_NEAR(0, tr.getBasis().getRow(2).getY(), 1e-12);
        EXPECT_NEAR(0, tr.getBasis().getRow(2).getZ(), 1e-12);

        EXPECT_NEAR(0, tr.getOrigin().getX(), 1e-6);
        EXPECT_NEAR(0, tr.getOrigin().getY(), 1e-6);
        EXPECT_NEAR(-equatorial_radius, tr.getOrigin().getZ(), 1e-6);
    }

    void checkTfEcefEnuForLat0Lon0(const tf2::Transform &tr) {
        EXPECT_NEAR(0, tr.getBasis().getRow(0).getX(), 1e-12);
        EXPECT_NEAR(0, tr.getBasis().getRow(0).getY(), 1e-12);
        EXPECT_NEAR(1, tr.getBasis().getRow(0).getZ(), 1e-12);
        EXPECT_NEAR(1, tr.getBasis().getRow(1).getX(), 1e-12);
        EXPECT_NEAR(0, tr.getBasis().getRow(1).getY(), 1e-12);
        EXPECT_NEAR(0, tr.getBasis().getRow(1).getZ(), 1e-12);
        EXPECT_NEAR(0, tr.getBasis().getRow(2).getX(), 1e-12);
        EXPECT_NEAR(1, tr.getBasis().getRow(2).getY(), 1e-12);
        EXPECT_NEAR(0, tr.getBasis().getRow(2).getZ(), 1e-12);

        EXPECT_NEAR(equatorial_radius, tr.getOrigin().getX(), 1e-6);
        EXPECT_NEAR(0, tr.getOrigin().getY(), 1e-6);
        EXPECT_NEAR(0, tr.getOrigin().getZ(), 1e-6);
    }

    void checkTfEnuEcefForLat0Lon45(const tf2::Transform &tr) {
        // X = [-0.707, 0, 0.707]^T
        // Y = [0.707, 0, 0.707]^T
        // Z = [0, 1, 0]^T
        EXPECT_NEAR(-half_power, tr.getBasis().getRow(0).getX(), 1e-12);
        EXPECT_NEAR(half_power, tr.getBasis().getRow(0).getY(), 1e-12);
        EXPECT_NEAR(0, tr.getBasis().getRow(0).getZ(), 1e-12);
        EXPECT_NEAR(0, tr.getBasis().getRow(1).getX(), 1e-12);
        EXPECT_NEAR(0, tr.getBasis().getRow(1).getY(), 1e-12);
        EXPECT_NEAR(1, tr.getBasis().getRow(1).getZ(), 1e-12);
        EXPECT_NEAR(half_power, tr.getBasis().getRow(2).getX(), 1e-12);
        EXPECT_NEAR(half_power, tr.getBasis().getRow(2).getY(), 1e-12);
        EXPECT_NEAR(0, tr.getBasis().getRow(2).getZ(), 1e-12);

        EXPECT_NEAR(0, tr.getOrigin().getX(), 1e-6);
        EXPECT_NEAR(0, tr.getOrigin().getY(), 1e-6);
        EXPECT_NEAR(-equatorial_radius, tr.getOrigin().getZ(), 1e-6);
    }

    void checkTfEcefEnuForLat0Lon45(const tf2::Transform &tr) {
        // Invert above
        EXPECT_NEAR(-half_power, tr.getBasis().getRow(0).getX(), 1e-12);
        EXPECT_NEAR(0, tr.getBasis().getRow(0).getY(), 1e-12);
        EXPECT_NEAR(half_power, tr.getBasis().getRow(0).getZ(), 1e-12);
        EXPECT_NEAR(half_power, tr.getBasis().getRow(1).getX(), 1e-12);
        EXPECT_NEAR(0, tr.getBasis().getRow(1).getY(), 1e-12);
        EXPECT_NEAR(half_power, tr.getBasis().getRow(1).getZ(), 1e-12);
        EXPECT_NEAR(0, tr.getBasis().getRow(2).getX(), 1e-12);
        EXPECT_NEAR(1, tr.getBasis().getRow(2).getY(), 1e-12);
        EXPECT_NEAR(0, tr.getBasis().getRow(2).getZ(), 1e-12);

        EXPECT_NEAR(
          equatorial_radius * half_power, tr.getOrigin().getX(), 1e-6);
        EXPECT_NEAR(
          equatorial_radius * half_power, tr.getOrigin().getY(), 1e-6);
        EXPECT_NEAR(0, tr.getOrigin().getZ(), 1e-6);
    }
};

TEST_F(rosTf2HelpersTest, Lat0Lon0Alt0Navsat) {
    sensor_msgs::NavSatFix datum_navsat;
    datum_navsat.latitude = 0;
    datum_navsat.longitude = 0;
    datum_navsat.altitude = 0;
    tf2::Transform result_enu_ecef = makeTf2ENUFromECEF(datum_navsat);
    tf2::Transform result_ecef_enu = makeTf2ECEFFromENU(datum_navsat);

    checkTfEnuEcefForLat0Lon0(result_enu_ecef);
    checkTfEnuEcefForLat0Lon0(result_ecef_enu.inverse());
    checkTfEcefEnuForLat0Lon0(result_ecef_enu);
    checkTfEcefEnuForLat0Lon0(result_enu_ecef.inverse());
}

TEST_F(rosTf2HelpersTest, Lat0Lon0Alt0LLH) {
    double datum_llh[3] = {0, 0, 0};
    tf2::Transform result_enu_ecef = makeTf2ENUFromECEF(datum_llh);
    tf2::Transform result_ecef_enu = makeTf2ECEFFromENU(datum_llh);

    checkTfEnuEcefForLat0Lon0(result_enu_ecef);
    checkTfEnuEcefForLat0Lon0(result_ecef_enu.inverse());
    checkTfEcefEnuForLat0Lon0(result_ecef_enu);
    checkTfEcefEnuForLat0Lon0(result_enu_ecef.inverse());
}

TEST_F(rosTf2HelpersTest, Lat0Lon0Alt0ECEF) {
    double datum_ecef[3] = {equatorial_radius, 0, 0};
    tf2::Transform result_enu_ecef = makeTf2ENUFromECEF(datum_ecef, false);
    tf2::Transform result_ecef_enu = makeTf2ECEFFromENU(datum_ecef, false);

    checkTfEnuEcefForLat0Lon0(result_enu_ecef);
    checkTfEnuEcefForLat0Lon0(result_ecef_enu.inverse());
    checkTfEcefEnuForLat0Lon0(result_ecef_enu);
    checkTfEcefEnuForLat0Lon0(result_enu_ecef.inverse());
}

TEST_F(rosTf2HelpersTest, Lat0Lon45Alt0Navsat) {
    sensor_msgs::NavSatFix datum_navsat;
    datum_navsat.latitude = 0;
    datum_navsat.longitude = 45;
    datum_navsat.altitude = 0;
    tf2::Transform result_enu_ecef = makeTf2ENUFromECEF(datum_navsat);
    tf2::Transform result_ecef_enu = makeTf2ECEFFromENU(datum_navsat);

    checkTfEnuEcefForLat0Lon45(result_enu_ecef);
    checkTfEnuEcefForLat0Lon45(result_ecef_enu.inverse());
    checkTfEcefEnuForLat0Lon45(result_ecef_enu);
    checkTfEcefEnuForLat0Lon45(result_enu_ecef.inverse());
}

TEST_F(rosTf2HelpersTest, Lat0Lon45Alt0LLH) {
    double datum_llh[3] = {0, 45, 0};

    tf2::Transform result_enu_ecef = makeTf2ENUFromECEF(datum_llh);
    tf2::Transform result_ecef_enu = makeTf2ECEFFromENU(datum_llh);

    checkTfEnuEcefForLat0Lon45(result_enu_ecef);
    checkTfEnuEcefForLat0Lon45(result_ecef_enu.inverse());
    checkTfEcefEnuForLat0Lon45(result_ecef_enu);
    checkTfEcefEnuForLat0Lon45(result_enu_ecef.inverse());
}

TEST_F(rosTf2HelpersTest, Lat0Lon45Alt0ECEF) {
    double datum_ecef[3] = {
      half_power * equatorial_radius, half_power * equatorial_radius, 0};

    tf2::Transform result_enu_ecef = makeTf2ENUFromECEF(datum_ecef, false);
    tf2::Transform result_ecef_enu = makeTf2ECEFFromENU(datum_ecef, false);

    checkTfEnuEcefForLat0Lon45(result_enu_ecef);
    checkTfEnuEcefForLat0Lon45(result_ecef_enu.inverse());
    checkTfEcefEnuForLat0Lon45(result_ecef_enu);
    checkTfEcefEnuForLat0Lon45(result_enu_ecef.inverse());
}


}  // namespace wave_spatial_utils
