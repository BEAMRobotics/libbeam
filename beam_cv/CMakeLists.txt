PROJECT(beam_cv)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    beam::calibration
    beam::containers
    beam::utils
    beam::defects
    Catch2::Catch2
    catkin::catkin
    Eigen3::Eigen
    OpenCV::OpenCV
    PCL::PCL
  SOURCES
    src/Utils.cpp
    src/RayCast.cpp
    src/geometry/Triangulation.cpp
    src/geometry/RelativePoseEstimator.cpp
    src/geometry/AbsolutePoseEstimator.cpp
)

############# Test files #############
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

add_executable(${PROJECT_NAME}_p3p_tests
  tests/p3p_test.cpp
)
target_include_directories(${PROJECT_NAME}_p3p_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_p3p_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

add_executable(${PROJECT_NAME}_debug
  tests/debug.cpp
)
target_include_directories(${PROJECT_NAME}_debug
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_debug
  ${PROJECT_NAME}
)

file(COPY tests/run_all_tests.bash
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

