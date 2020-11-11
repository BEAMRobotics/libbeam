PROJECT(beam_cv)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    catkin::catkin
    PCL::PCL
    beam::calibration
    beam::utils
    beam::depth
    OpenCV::OpenCV
    Eigen3::Eigen
  SOURCES
    src/Utils.cpp
    src/geometry/Triangulation.cpp
    src/geometry/RelativePoseEstimator.cpp
    src/geometry/AbsolutePoseEstimator.cpp
    src/descriptors/BRISKDescriptor.cpp
    src/descriptors/ORBDescriptor.cpp
    src/descriptors/SIFTDescriptor.cpp
    src/detectors/ORBDetector.cpp
    src/detectors/FASTDetector.cpp
    src/detectors/SIFTDetector.cpp
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

add_executable(${PROJECT_NAME}_geometry_tests
  tests/geometry_tests.cpp
)

target_include_directories(${PROJECT_NAME}_geometry_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_geometry_tests
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

