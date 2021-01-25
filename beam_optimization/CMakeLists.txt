PROJECT(beam_optimization)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    beam::utils
    beam::calibration
    Boost::boost
    Catch2::Catch2
    catkin::catkin
    Eigen3::Eigen
    PCL::PCL
  SOURCES
)

################ test libraries ##############

add_library(test_utils STATIC tests/src/test_util.cpp)


target_link_libraries(test_utils
  beam::calibration
  beam::utils
)

target_include_directories(test_utils
  PUBLIC
    tests/include
    ${PCl_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

################ tests ##################

add_executable(${PROJECT_NAME}_ceres_ladybug_tests
  tests/src/ladybug_reprojection_tests.cpp
)

target_include_directories(${PROJECT_NAME}_ceres_ladybug_tests
  PUBLIC
    include/beam_optimization
    ${catkin_INCLUDE_DIRS}
    ${PCl_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_ceres_ladybug_tests
  ${PROJECT_NAME}
  Catch2::Catch2
  ${CERES_LIBRARIES}
  ${PCL_LIBRARY_DIRS}
  test_utils
  beam::utils
)

add_executable(${PROJECT_NAME}_ceres_radtan_tests
  tests/src/radtan_reprojection_tests.cpp
)

target_include_directories(${PROJECT_NAME}_ceres_radtan_tests
  PUBLIC
    include/beam_optimization
    ${catkin_INCLUDE_DIRS}
    ${PCl_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_ceres_radtan_tests
  ${PROJECT_NAME}
  Catch2::Catch2
  ${CERES_LIBRARIES}
  ${PCL_LIBRARY_DIRS}
  test_utils
  beam::utils
)

file(COPY tests/run_all_tests.bash
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

message (${CMAKE_CURRENT_BINARY_DIR})
