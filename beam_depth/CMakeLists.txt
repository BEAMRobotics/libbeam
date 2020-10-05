PROJECT(beam_depth)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    catkin::catkin
    PCL::PCL
    beam::calibration
    beam::containers
    beam::utils
    beam::cv
    OpenCV::OpenCV
    Eigen3::Eigen
  SOURCES
    src/Utils.cpp
    src/DepthMap.cpp
    src/DepthCompletion.cpp
)

############# Test files #############

add_executable(${PROJECT_NAME}_map_tests
  tests/depthmap_test.cpp
)

target_include_directories(${PROJECT_NAME}_map_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_map_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

add_executable(${PROJECT_NAME}_kitti
  tests/kitti.cpp
)
target_include_directories(${PROJECT_NAME}_kitti
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_kitti
  ${PROJECT_NAME}
  Eigen3::Eigen
  ${CERES_LIBRARIES}
)

file(COPY tests/run_all_tests.bash
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)
