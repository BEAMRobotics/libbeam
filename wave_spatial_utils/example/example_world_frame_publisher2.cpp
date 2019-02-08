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
 * File: world_frame_conversions.cpp
 * Desc: Implementation file for world frame conversion functions
 * Auth: Michael Smart <michael.smart@uwaterloo.ca>
 *
 * ############################################################################
*/

#include "wave_spatial_utils/world_frame_conversions.hpp"

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "wave_spatial_utils_tf_broadcaster_example2");

    double equatorial_earth_radius = 6.378137e6;

    // Here, map is defined by an LLA datum
    sensor_msgs::NavSatFix map_datum;
    map_datum.latitude = 0;
    map_datum.longitude = 0;
    map_datum.altitude = 0;

    // Get the tf2::Transform taking data from ENU to ECEF at the map datum
    // the datum can be a NavSatFix message
    tf2::Transform tf_earth_map =
      wave_spatial_utils::makeTf2ECEFFromENU(map_datum);

    // The datum can be sent as ECEF points with the extra "false" argument
    // to indicate the provided datum is *not* LLA.
    //
    // Let's say the odom datum is 1km in ECEF's Y direction from map's datum
    double odom_datum_ecef[3] = {equatorial_earth_radius, 1000.0, 0};
    tf2::Transform tf_earth_odom =
      wave_spatial_utils::makeTf2ECEFFromENU(odom_datum_ecef, false);

    // How do we get tf_map_odom?
    // tf_map_odom = tf_map_earth * tf_earth_odom
    // and tf_map_earth = inverse(tf_earth_map)
    tf2::Transform tf_map_odom = tf_earth_map.inverse() * tf_earth_odom;

    tf2_ros::TransformBroadcaster br;
    ros::Rate loop_rate(1);

    while (ros::ok()) {
        // Create the stamped transforms for the tree and publish them:
        //
        // NOTE: Order of frame arguments is - FRAME, CHILD
        // i.e. TARGET, SOURCE in terms of how the transform converts points
        geometry_msgs::TransformStamped T_earth_map;
        T_earth_map.header.stamp = ros::Time::now();
        T_earth_map.header.frame_id = "earth";
        T_earth_map.child_frame_id = "map";
        T_earth_map.transform = tf2::toMsg(tf_earth_map);
        br.sendTransform(T_earth_map);
        geometry_msgs::TransformStamped T_map_odom;
        T_map_odom.header.stamp = ros::Time::now();
        T_map_odom.header.frame_id = "map";
        T_map_odom.child_frame_id = "odom";
        T_map_odom.transform = tf2::toMsg(tf_map_odom);
        br.sendTransform(T_map_odom);
        ros::spinOnce();
        loop_rate.sleep();
    }
};
