PROJECT(beam_mapping)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    beam::utils
    beam::calibration
    beam::filtering
    rosbag::rosbag
  SOURCES
    src/MapBuilder.cpp
    src/Poses.cpp
    src/Utils.cpp
)

################ tests ##################

add_executable(${PROJECT_NAME}_map_builder_tests
  tests/MapBuilderTest.cpp
)

target_include_directories(${PROJECT_NAME}_map_builder_tests
  PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME}_map_builder_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

add_executable(${PROJECT_NAME}_poses_test
  tests/PosesTest.cpp
)

target_include_directories(${PROJECT_NAME}_poses_test
  PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME}_poses_test
  ${PROJECT_NAME}
  Catch2::Catch2
)

file(COPY tests/run_all_tests.bash
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)
