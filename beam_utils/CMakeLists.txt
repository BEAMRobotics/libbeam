PROJECT(beam_utils)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    OpenCV::OpenCV
    Eigen3::Eigen
    roscpp::roscpp
    PCL::PCL
    Boost::boost
    nlohmann_json::nlohmann_json
    gflags
  SOURCES
    src/math.cpp
    src/time.cpp
    src/angles.cpp
    src/gflags.cpp
    src/visualizer.cpp
    src/filesystem.cpp
    src/roots.cpp
    src/polynomial.cpp
    src/optional.cpp
    src/pointclouds.cpp
    src/simple_path_generator.cpp
)

add_executable(${PROJECT_NAME}_unit_tests
  tests/angles_test.cpp
  tests/time_test.cpp
  tests/math_test.cpp
  tests/filesystem_test.cpp
  tests/utils_tests_main.cpp
)
target_include_directories(${PROJECT_NAME}_unit_tests
  PUBLIC
  ../include
)
target_link_libraries(${PROJECT_NAME}_unit_tests
  ${PROJECT_NAME}
    Catch2::Catch2
)

file(COPY tests/run_all_tests.bash
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)
