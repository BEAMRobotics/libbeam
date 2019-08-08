PROJECT(beam_matching)

SET(CMAKE_CXX_STANDARD 17)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    beam::utils
    Eigen3::Eigen
    Catch2::Catch2
    Boost::boost
    PCL::PCL
    nlohmann_json::nlohmann_json
  SOURCES
    src/IcpMatcher.cpp
)

# Unit tests
#IF(BUILD_TESTING)
#    WAVE_ADD_TEST(${PROJECT_NAME}_tests
#        tests/icp_tests.cpp
#        tests/ndt_tests.cpp
#        tests/gicp_tests.cpp
#        tests/multi_matcher_tests.cpp)

#WAVE_ADD_TEST(
#    ${PROJECT_NAME}_viz_tests
#    tests/pointcloud_display_tests.cpp
#    DISABLED  # probably requires display to work properly
#)

#WAVE_ADD_TEST(${PROJECT_NAME}_ground_segmentation_tests
#    tests/ground_segmentation_test.cpp
#    DISABLED  # Uses viewer to check results
#)

add_executable(${PROJECT_NAME}_icp_tests
  tests/icp_tests.cpp
)

target_include_directories(${PROJECT_NAME}_icp_tests
  PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME}_icp_tests
    ${PROJECT_NAME}
    beam::utils
    Eigen3::Eigen
    Catch2::Catch2
)

#TARGET_LINK_LIBRARIES(
#    ${PROJECT_NAME}_viz_tests
#    ${PROJECT_NAME}
#    beam_utils
#)

#TARGET_LINK_LIBRARIES(
#    ${PROJECT_NAME}_ground_segmentation_tests
#    ${PROJECT_NAME}
#    beam_utils
#)

#TARGET_LINK_LIBRARIES(
#    ${PROJECT_NAME}_viz_tests
#    ${PROJECT_NAME}
#    beam_utils
#)

# Copy the test data
file(COPY tests/data tests/config DESTINATION ${PROJECT_BINARY_DIR}/tests)
#ENDIF(BUILD_TESTING)
