# wave_spatial_utils

## Overview

The `wave_spatial_utils` catkin package is a library housing several spatial
utilities including angle wrapping and world frame conversions between ECEF,
LLH, and local ENU coordinates and frames.

The world frame conversion functions simply wrap calls to GeographicLib. For
usage that is large scale or does not fit the provided interfaces it is
recommended to just use GeographicLib directly.

## Example

The package contains an example program (source under example/) illustrating how
to use the functions in the library to produce and publish transforms between
ECEF and ENU frames with the ROS `tf` and `tf2` system.
