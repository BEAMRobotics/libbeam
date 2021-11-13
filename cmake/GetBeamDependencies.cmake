# Find all dependencies here, and ensure they have IMPORTED targets
# Require Eigen 3.2.92, also called 3.3 beta-1, since it's in xenial
FIND_PACKAGE(Eigen3 3.2.92 REQUIRED)
FIND_PACKAGE(Boost 1.58.0 REQUIRED system filesystem)
FIND_PACKAGE(PCL 1.11.1 REQUIRED COMPONENTS
    common filters registration kdtree search io visualization
    surface segmentation features)
FIND_PACKAGE(Catch2 REQUIRED)
FIND_PACKAGE(nlohmann_json 3.2.0 REQUIRED)
FIND_PACKAGE(roscpp REQUIRED)
FIND_PACKAGE(tf2 REQUIRED)
FIND_PACKAGE(rosbag REQUIRED)
		
# Ceres is only required for certain modules. Let this be optional
FIND_PACKAGE(Ceres 1.12 QUIET)

# OpenCV4 is only required when building: cv, colorize, containers, 
# defects, depth. Let the user decide when to use the default opencv
IF(NOT CMAKE_IGNORE_BEAM_OPENCV4)
    FIND_PACKAGE(OpenCV 4.5.2 REQUIRED) 
    INCLUDE(${CMAKE_CURRENT_LIST_DIR}/ImportOpenCV.cmake)
ELSE()
    FIND_PACKAGE(OpenCV)
    INCLUDE(${CMAKE_CURRENT_LIST_DIR}/ImportOpenCV.cmake)
ENDIF()

# Where dependencies do not provide imported targets, define them
INCLUDE(${CMAKE_CURRENT_LIST_DIR}/ImportEigen3.cmake)
INCLUDE(${CMAKE_CURRENT_LIST_DIR}/ImportBoost.cmake)
INCLUDE(${CMAKE_CURRENT_LIST_DIR}/ImportPCL.cmake)
INCLUDE(${CMAKE_CURRENT_LIST_DIR}/ImportRosdeps.cmake)
INCLUDE(${CMAKE_CURRENT_LIST_DIR}/ImportCeres.cmake)

IF(BUILD_LADYBUG)
    FIND_PACKAGE(Ladybug REQUIRED)
    INCLUDE(${CMAKE_CURRENT_LIST_DIR}/FindLadybug.cmake)
ENDIF()
