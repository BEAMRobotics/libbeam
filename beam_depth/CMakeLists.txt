PROJECT(beam_depth)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    beam::calibration
    beam::colorize
    beam::utils
    beam::cv
  SOURCES
    src/Utils.cpp
    src/DepthMap.cpp
    src/DepthCompletion.cpp
)

############# Test files #############

add_executable(${PROJECT_NAME}_map_tests
  tests/depth_map_test.cpp
)

target_include_directories(${PROJECT_NAME}_map_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_map_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

add_executable(${PROJECT_NAME}_utils_tests
  tests/utils_test.cpp
)

target_include_directories(${PROJECT_NAME}_utils_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_utils_tests
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
)

file(COPY tests/run_all_tests.bash
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)
