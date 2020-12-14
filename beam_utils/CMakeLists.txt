PROJECT(beam_utils)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    Eigen3::Eigen
    catkin::catkin
    yaml-cpp
    PCL::PCL
    OpenCV::OpenCV
    gflags
    Boost::boost
  SOURCES
    src/math.cpp
    src/time.cpp
    src/angles.cpp
    src/gflags.cpp
    src/filesystem.cpp
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
  Catch2::Catch2
  ${PROJECT_NAME}
)

file(COPY tests/run_all_tests.bash
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)
