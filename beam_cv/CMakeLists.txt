PROJECT(beam_cv)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    beam::calibration
    beam::containers
    beam::utils
    beam::depth
    OpenCV::OpenCV
    Eigen3::Eigen
    OpenCV::OpenCV
    PCL::PCL
    ${TORCH_LIBRARIES}
  SOURCES
    src/Utils.cpp
    src/Raycast.cpp
    src/Tracker.cpp
    src/geometry/Triangulation.cpp
    src/geometry/RelativePoseEstimator.cpp
    src/geometry/AbsolutePoseEstimator.cpp
    src/descriptors/BRISKDescriptor.cpp
    src/descriptors/ORBDescriptor.cpp
    src/descriptors/SIFTDescriptor.cpp
    src/descriptors/SuperPointDescriptor.cpp
    src/detectors/ORBDetector.cpp
    src/detectors/FASTDetector.cpp
    src/detectors/SIFTDetector.cpp
    src/detectors/SuperPointDetector.cpp
    src/matchers/FLANNMatcher.cpp
    src/models/SuperPointModel.cpp
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

add_executable(${PROJECT_NAME}_superpoint_tests
  tests/superpoint_tests.cpp
)

target_include_directories(${PROJECT_NAME}_superpoint_tests
  PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME}_superpoint_tests
  ${PROJECT_NAME}
  Catch2::Catch2
  ${TORCH_LIBRARIES}
)

add_executable(${PROJECT_NAME}_feature_tests
  tests/feature_tests.cpp
)

target_include_directories(${PROJECT_NAME}_feature_tests
  PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME}_feature_tests
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

