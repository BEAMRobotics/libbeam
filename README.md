![libbeam_logo](https://github.com/BEAMRobotics/libbeam/assets/25440002/3517e87e-13bf-4ca2-8a1b-a50dbcf3c872)

# libbeam
[![Build Status](https://travis-ci.com/BEAMRobotics/libbeam.svg?token=zshhVvp9R3DJ7YGGYs6z&branch=master)](https://travis-ci.com/BEAMRobotics/libbeam)

libbeam is a library developed by the former [SDIC Lab](http://www.civil.uwaterloo.ca/snarasim/) at the University of Waterloo, now maintained by the [SRI Lab](https://sri-lab.seas.ucla.edu/) at UCLA. The goal of this library was to have a place for all re-usable code for our project on infrastructure inspection using robotics, nicknamed Beam Robotics. This project required code to be built for an end-to-end inspection pipeline include robot sensor calibration, SLAM, ML-based defect detection, image projection and back-projection, pointcloud colorization, pointcloud map generation and filtering, and more.

For questions, email one of the following maintainers:

* Alex Thoms: adthoms@uwaterloo.ca
* Nick Charron: nicholas.c.charron@gmail.com

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

## Modules

We will briefly go over all available modules within libbeam:

### beam_calibration 

This module contains two parts: 

1. camera models: We create an extensible class of cameras models with common implementations including pinhole, double sphere, Kanala Brandt, Cataditrophic and the Ladybug Camera (not this relies on the ladybug SDK which only works on Ubuntu 16 - so this is deprecated). We also provide a tool for converting images between camera models. 
2. TfTree: for storing extrinsic calibrations, we create a TfTree object which is a wrapper around tf2::BufferCore with some useful extensions. For example, we allow different inputs and output (i.e., Eigen). We also allow for the case of inputting a frame that has two parents, which tf2 does not allow for. Lastly, we provide a simple way to read and write an extrinsics calibration tree from a json file.

### beam_colorize

This contains the tools needed to colorize a pointcloud map with image data, including a projection method and a raytrace method.

### beam_containers

This is a utilities class for storing different types of containers that are useful for other modules. For example, ImageBridge is a data structure for storing all data that was extracted from an image of a bridge including defect masks, IR and RGB raw images, and more. We also have LandmarkContainer and LandmarkMeasurement which are used for tasks such as SLAM and relocalization. And finally, we have a custom point type, PointBridge, that holds all defect information about a point on a bridge from an automated inspection (e.g., belonging to crack or delam)

### beam_cv

This module has all computer vision code needed for all other modules, or SLAM/Reloc code created outside libbeam. The major components are:

* descriptors: abstract class definition for a descriptor, with several implementations (i.e., BEBLID, BRISK, ORB, SIFT)
* detectors: abstract class definition for a detector, with several implementations (i.e., FASY, GFTT, ORB, SIFT)
* geometry: some useful CV geometry algorithms including absolute pose estimation, non-linear pose refinement, relative pose estimation, and point triangulation
* matchers: abstract class definition for methods of matching image features between image pairs, with specific implementation including: brute force, FLANN
* trackers: abstract class definition for methods of tracking features between subsequent images in an image sequence, with specific implementations including: KLT and descriptor matching
* Others: common conversions and utilities

### beam_defects

This module has classes that store different defect classes that have been back projected from an image to a pointcloud. We define an abstract defect class with common functionalities needed for each defect type (e.g., get size), with implementations of crack, spall, and delamination.

### beam_depth

This module contains a class representing depth maps, provifing the ability to extract them easily and perform operations on them given a camera model, point cloud and image. For example, we provide a depth completion algorithm to densify sparse depth maps produces from devices such as a lidar.

### beam_filtering

This module contains pointcloud filtering functionality. We define our own abstract class to give our specific desired user interface, and implement several functions including some that are direct wrapers over PCL, as well as custom filters and variations of PCL filters.

### beam_mapping

This module contains code that is useful for creating pointcloud maps from a set of poses and 3D data. We provide two classes:

1. MapBuilder: this takes a bag with 3D data, extrinsics, and a pose file and creates a final pointcloud map
2. Poses: this is a useful class for creating, storing, reading, and writing poses from say some SLAM output

See: https://github.com/nickcharron/3d_map_builder

### beam_matching

This module performs scan matching (or scan registration, or point set registration) between pcl pointclouds. We define an abstract class with an interface designed for our needs, and implement various matching techniques including: [LOAM](https://www.ri.cmu.edu/pub_files/2014/7/Ji_LidarMapping_RSS2014_v8.pdf), iterative closest point, generalized iterative closest point, and the Normal Distributions Transform.

### beam_optimization

This module has tools for optimization that are useful for other modules, and inspection tasks including SLAM. We provide some ceres cost functions (i.e., camera reprojection cost, point to line cost, point to plane cost, pose prior cose) as well as a class for helping to load ceres params.

### beam_utils

This module is a general utilities module that has shared code between two or many of the other modules. Example types of utilities are:

* angles utils
* math utils
* time utils
* pointcloud utils
* filesystem utils
* logging utils
