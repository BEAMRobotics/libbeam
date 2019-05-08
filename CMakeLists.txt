CMAKE_MINIMUM_REQUIRED(VERSION 3.14)
PROJECT(beam)

# Package version, used when other projects FIND_PACKAGE(beam <version>)
SET(BEAM_PACKAGE_VERSION 0.1.0)

# Compiler settings for all targets
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_POSITION_INDEPENDENT_CODE ON)
ADD_COMPILE_OPTIONS(-Wall -Wextra -Wno-strict-overflow)

# Default to Release build type, otherwise some modules will be slow
IF(NOT CMAKE_BUILD_TYPE)
   MESSAGE(STATUS "Using 'Release' build type as CMAKE_BUILD_TYPE is not set")
   # Set the value but keep it as an option in CMake GUI
   SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
   SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
       "None" "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
ENDIF()

# CMake modules
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
INCLUDE(CMakePackageConfigHelpers)
INCLUDE(cmake/BeamHelpers.cmake)

# User options
OPTION(BUILD_SHARED_LIBS "Build shared instead of static libraries" OFF)
OPTION(EXPORT_BUILD
 "Add this build directory to CMake's user package registry.\
 Allows the package to be found without install." OFF)


# Find all dependencies here, and ensure they have IMPORTED targets
# Require Eigen 3.2.92, also called 3.3 beta-1, since it's in xenial
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(Boost 1.58.0 COMPONENTS system filesystem)
FIND_PACKAGE(PCL 1.8 REQUIRED COMPONENTS
    common filters registration kdtree search io visualization
    surface segmentation)
FIND_PACKAGE(kindr)
FIND_PACKAGE(OpenCV 3.2.0 REQUIRED COMPONENTS)
FIND_PACKAGE(yaml-cpp REQUIRED)
FIND_PACKAGE(Ceres 1.11)
FIND_PACKAGE(GTSAM)
FIND_PACKAGE(GeographicLib 1.49)
FIND_PACKAGE(catkin REQUIRED COMPONENTS tf2 tf2_eigen sensor_msgs rostime
             roscpp std_msgs cv_bridge)
FIND_PACKAGE(Catch2 REQUIRED)
FIND_PACKAGE(nlohmann_json 3.2.0 REQUIRED)
FIND_PACKAGE(Ladybug)
message(STATUS "Ladybug found ${Ladybug_FOUND}")
message(STATUS "Ladybug include dirs: ${Ladybug_INCLUDE_DIRS}")


# Where dependencies do not provide IMPORTED targets, define them
INCLUDE(cmake/ImportEigen3.cmake)
INCLUDE(cmake/ImportBoost.cmake)
INCLUDE(cmake/ImportPCL.cmake)
INCLUDE(cmake/ImportKindr.cmake)
INCLUDE(cmake/ImportYaml-cpp.cmake)
INCLUDE(cmake/ImportCeres.cmake)
INCLUDE(cmake/ImportCatkin.cmake)
INCLUDE(cmake/ImportOpenCV.cmake)

# Add a special "beam" target including all modules
# The BEAM_ADD_LIBRARY helper in BeamHelpers.cmake will add each module to this
ADD_LIBRARY(beam INTERFACE)
INSTALL(TARGETS beam EXPORT beamTargets)

# Add each module to the project
# Modules with missing dependencies are not built, and circular dependencies are
# not supported. Thus modules must be listed after their dependencies, for now.
ADD_SUBDIRECTORY(beam_calibration)
ADD_SUBDIRECTORY(beam_colorize)
ADD_SUBDIRECTORY(beam_defects)
ADD_SUBDIRECTORY(beam_utils)
ADD_SUBDIRECTORY(beam_containers)

# Documentation
SET(BEAM_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
SET(BEAM_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})

# This is where .cmake files will be installed (default under share/)
SET(INSTALL_CMAKE_DIR "${CMAKE_INSTALL_DATADIR}/beam/cmake" CACHE PATH
    "Installation directory for CMake files")

# Add an export file listing all targets
EXPORT(EXPORT beamTargets NAMESPACE beam:: FILE beamTargets.cmake)

# Install the export file
INSTALL(EXPORT beamTargets NAMESPACE beam:: DESTINATION "${INSTALL_CMAKE_DIR}")

# Create the beamConfig and beamConfigVersion files

# Generate the Config file for the install tree
SET(BEAM_EXTRA_CMAKE_DIR "cmake")
CONFIGURE_PACKAGE_CONFIG_FILE(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/beamConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/beamConfig.cmake"
    INSTALL_DESTINATION "${INSTALL_CMAKE_DIR}")

# Generate the Version file
WRITE_BASIC_PACKAGE_VERSION_FILE(beamConfigVersion.cmake
    VERSION ${BEAM_PACKAGE_VERSION}
    COMPATIBILITY SameMajorVersion)

# Install the Config and ConfigVersion files
INSTALL(FILES
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/beamConfig.cmake"
    "${PROJECT_BINARY_DIR}/beamConfigVersion.cmake"
    DESTINATION "${INSTALL_CMAKE_DIR}")

# Install our import scripts so we can use them in the Config script. That way,
# other projects using libbeam will get transient dependencies automatically
INSTALL(DIRECTORY cmake/
    DESTINATION "${INSTALL_CMAKE_DIR}/cmake"
    FILES_MATCHING PATTERN "Import*.cmake" PATTERN "Find*.cmake")

IF(EXPORT_BUILD)
    # Export this build so the package can be found through CMake's registry
    # without being installed.
    EXPORT(PACKAGE beam)

    # Generate the Config file for the build tree
    # It differs from the installed Config file in where our bundled
    # Import*.cmake scripts are included from. Here, use the relative path from
    # the build directory to this source directory
    FILE(RELATIVE_PATH path_to_beam_source
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR})
    SET(BEAM_EXTRA_CMAKE_DIR "${path_to_beam_source}cmake")
    CONFIGURE_PACKAGE_CONFIG_FILE(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/beamConfig.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/beamConfig.cmake"
        INSTALL_DESTINATION "${INSTALL_CMAKE_DIR}")
ENDIF(EXPORT_BUILD)
