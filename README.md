# libbeam
[![Build Status](https://travis-ci.com/BEAMRobotics/libbeam.svg?token=zshhVvp9R3DJ7YGGYs6z&branch=master)](https://travis-ci.com/BEAMRobotics/libbeam)

libbeam is a library developed by the former [SDIC Lab](http://www.civil.uwaterloo.ca/snarasim/) at the University of Waterloo, now maintained by the [SRI Lab](https://sri-lab.seas.ucla.edu/) at UCLA. The goal of this library was to have a place for all re-usable code for our project on infrastructure inspection using robotics, nicknamed Beam Robotics. This project required code to be built for an end-to-end inspection pipeline include robot sensor calibration, SLAM, ML-based defect detection, image projection and back-projection, pointcloud colorization, pointcloud map generation and filtering, and more.

For questions, email one of the following maintainers:

Alex Thoms: adthoms@uwaterloo.ca
Nick Charron: nicholas.c.charron@gmail.com

## Installing 

We provide an install script in scripts/install.bash

The install script will install libbeam and all dependencies to your system if they are not currently available including:

* ROS (and all required ROS depenencies)
* Catch2
* Eigen3
* Ceres
* PCL (min version 1.11.1)
* OpenCV4
* Gflags
* nlohmann json

Since we know not all of you will need all modules, and therefore you will not need all dependencies, we provide some parameters that can disable some of the install:

* libbeam depends on OpenCV4 for certain modules (e.g., beam_cv), however, if you are not using all modules, you may be able to get away with older versions (i.e., the version install by ROS). To disable OpenCV4 install set the parameter INSTALL_OPENCV4=0
* You may also chose to not install ceres if you are using modules that do not depend on ceres, by setting INSTALL_CERES=0. Currently, the modules that depend on ceres are: beam_optimzation, beam_cv, and beam_matching 
* You may chose to install libbeam and certain dependencies locally by setting the appropriate params (INSTALL_LIBBEAM_LOCALLY, INSTALL_OPENCV4_LOCALLY). Installing these locally means that we assume libbeam has been cloned to some catkin workspace (e.g., ~/catkin_ws/src/) and therefore we will not run make install on any of those packages. Instead, you will need to run catkin build in the workspace after this script has completed (IF SO, MAKE SURE YOU SET THE CMAKE VARIABLES PROPERLY - see below). This is recommended to avoid any versioning conflicts.
* You can disable any module by setting the appropriate parameter to true, e.g., IGNORE_BEAM_COLORIZE=1 to not build the beam_colorize module. Note that we do not check dependencies before trying to build. Therefore if you disable a module and try to build another module that depends on it, the build will crash. You will need to check the dependencies in each module's CMakeLists file.

If installing without the script, or if you installed using the script and set INSTALL_LIBBEAM_LOCALLY, you need to set the cmake params accodingly to not build all libbeam modules: E.g. -DCMAKE_IGNORE_BEAM_COLORIZE=1 to disable the beam_colorize module. 

Additionally, if you want to try to build with your current version of OpenCV, then set -DCMAKE_IGNORE_BEAM_OPENCV4=1. If you chose to not install Ceres, no parameters need to be set, it should work as long as you've disabled all modules that depend on Ceres (i.e., beam_optimzation, beam_cv, and beam_matching)

Here is an example build procedure if you have already install all dependencies, and you want to disable beam_colorize module:

```
mkdir -d catkin_ws/src
cd catkin_ws
catkin build
cd src
git clone --depth 1 -b v1.0.0 https://github.com/BEAMRobotics/libbeam.git
catkin build -DCMAKE_IGNORE_BEAM_COLORIZE=1
```



## Example Usage:

To use this library inside your program, add the following to your CMakeLists.txt file:

```
FIND_PACKAGE(beam REQUIRED utils [ADD ANY OTHER MODULES NEEDED])
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_node
	[...]
	beam::beam
	[...]
)
```

Then you can include the headers in your .cpp and .hpp files, e.g.:

`#include <beam_utils/math.h>`

For a simple example use of libbeam which only uses certain modules, see: https://github.com/nickcharron/3d_map_builder
