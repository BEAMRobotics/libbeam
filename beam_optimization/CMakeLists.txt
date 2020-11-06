PROJECT(beam_optimization)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    beam::utils
    beam::calibration
    Boost::boost
    Catch2::Catch2
    catkin::catkin
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    PCL::PCL
  SOURCES
)

################ test libraries ##############

add_library(util STATIC tests/util.cpp)
  
add_library(visualizer STATIC tests/visualizer.cpp)

target_link_libraries(visualizer 
  beam::matching
  beam::filtering
  ${PCl_LIBRARIES}
)

target_link_libraries(util
  beam::calibration
)



include_directories(
  include/beam_optimization
  ${catkin_INCLUDE_DIRS}
  ${PCl_INCLUDE_DIRS}
)


link_directories(${PROJECT_NAME}
  include
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

################ tests ##################

add_executable(${PROJECT_NAME}_ceres_lb_tests
  tests/ladybug_reprojection_tests.cpp
)

target_include_directories(${PROJECT_NAME}_ceres_lb_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_ceres_lb_tests
  ${PROJECT_NAME}
  Catch2::Catch2
  ${CERES_LIBRARIES}
  util
  visualizer
)

add_executable(${PROJECT_NAME}_ceres_rt_tests
  tests/radtan_reprojection_tests.cpp
)

target_include_directories(${PROJECT_NAME}_ceres_rt_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_ceres_rt_tests
  ${PROJECT_NAME}
  Catch2::Catch2
  ${CERES_LIBRARIES}
  util
  visualizer
)

file(COPY tests/run_all_tests.bash
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

message (${CMAKE_CURRENT_BINARY_DIR})
