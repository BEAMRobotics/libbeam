cmake_minimum_required(VERSION 2.8.3)
project(wave_spatial_utils)

# Use C++11
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

find_package(Eigen3 REQUIRED)
find_package (GeographicLib 1.49 REQUIRED)
add_definitions (${GeographicLib_DEFINITIONS})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(catkin_deps
    roscpp
    sensor_msgs
    geometry_msgs
    tf
    tf2
    tf2_ros)

find_package(catkin REQUIRED COMPONENTS ${catkin_deps})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    DEPENDS GeographicLib
    CATKIN_DEPENDS ${catkin_deps}
)

###########
## Build ##
###########

include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${GeographicLib_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR})

include_directories(include)

# C++ library
add_library(${PROJECT_NAME}
    src/angles.cpp
    src/world_frame_conversions.cpp
    src/pose_cov_comp.cpp)

target_link_libraries(${PROJECT_NAME}
    ${GeographicLib_LIBRARIES})

# Example publisher
add_executable(${PROJECT_NAME}_tf_broadcaster_example
    example/example_world_frame_publisher.cpp)
target_link_libraries(${PROJECT_NAME}_tf_broadcaster_example
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    ${GeographicLib_LIBRARIES})

add_executable(${PROJECT_NAME}_tf2_broadcaster_example
example/example_world_frame_publisher2.cpp)
target_link_libraries(${PROJECT_NAME}_tf2_broadcaster_example
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    ${GeographicLib_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_tf_broadcaster_example
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_tf2_broadcaster_example
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp")

#############
## Testing ##
#############

# Add gtest unit testing
if (CATKIN_ENABLE_TESTING)
    #### Explicitly add test .cpp files with the test executable target called ${PROJECT_NAME}-test
    catkin_add_gtest(${PROJECT_NAME}-test
            test/test_${PROJECT_NAME}.cpp
            test/test_angles.cpp
            test/test_ecef_llh_point_conversions.cpp
            test/test_ecef_enu_transforms.cpp
            test/test_enu_llh_point_conversions.cpp
            test/test_ros_fix_point_conversions.cpp
            test/test_ros_tf_helpers.cpp
            test/test_ros_tf2_helpers.cpp
            test/test_pose_cov_comp.cpp)
    #### Link test executable against non-ROS implementation library
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} ${PROJECT_NAME} ${GeographicLib_LIBRARIES})
endif(CATKIN_ENABLE_TESTING)
