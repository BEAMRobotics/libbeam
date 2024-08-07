PROJECT(beam_cv)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    beam::calibration
    beam::containers
    beam::utils
    beam::optimization
    DBoW3
  SOURCES
    src/Utils.cpp
    src/Raycast.cpp
    src/OpenCVConversions.cpp
    src/ImageDatabase.cpp
    src/geometry/Triangulation.cpp
    src/geometry/RelativePoseEstimator.cpp
    src/geometry/AbsolutePoseEstimator.cpp
    src/geometry/PoseRefinement.cpp
    src/descriptors/Descriptor.cpp
    src/descriptors/BRISKDescriptor.cpp
    src/descriptors/BEBLIDDescriptor.cpp
    src/descriptors/ORBDescriptor.cpp
    src/descriptors/SIFTDescriptor.cpp
    src/detectors/Detector.cpp
    src/detectors/ORBDetector.cpp
    src/detectors/FASTDetector.cpp
    src/detectors/FASTSSCDetector.cpp
    src/detectors/SIFTDetector.cpp
    src/detectors/GFTTDetector.cpp
    src/matchers/Matcher.cpp
    src/matchers/FLANNMatcher.cpp
    src/matchers/BFMatcher.cpp
    src/trackers/Tracker.cpp
    src/trackers/DescMatchingTracker.cpp
    src/trackers/KLTracker.cpp
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

add_executable(${PROJECT_NAME}_pose_refinement_tests
  tests/pose_refinement_tests.cpp
)

target_include_directories(${PROJECT_NAME}_pose_refinement_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_pose_refinement_tests
  ${PROJECT_NAME}
  Catch2::Catch2
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

add_executable(${PROJECT_NAME}_tracker_tests
  tests/tracker_tests.cpp
)

target_include_directories(${PROJECT_NAME}_tracker_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_tracker_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

file(COPY tests/run_all_tests.bash
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

