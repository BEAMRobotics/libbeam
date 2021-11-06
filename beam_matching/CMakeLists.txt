PROJECT(beam_matching)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    beam::utils
    beam::optimization
  SOURCES
    src/IcpMatcher.cpp
    src/NdtMatcher.cpp
    src/GicpMatcher.cpp
    src/LoamMatcher.cpp
    src/loam/LoamPointCloud.cpp
    src/loam/LoamFeatureExtractor.cpp
    src/loam/LoamScanRegistration.cpp
)

##################### TESTS #########################
add_executable(${PROJECT_NAME}_loam_gtests 
  tests/loam_gtests.cpp
)
target_link_libraries(${PROJECT_NAME}_loam_gtests 
  ${PROJECT_NAME}
  gtest_main
)
add_test(NAME ${PROJECT_NAME}_loam_gtests 
  COMMAND ${PROJECT_NAME}_loam_gtests
)

add_executable(${PROJECT_NAME}_icp_gtests 
  tests/icp_gtests.cpp
)
target_link_libraries(${PROJECT_NAME}_icp_gtests 
  ${PROJECT_NAME}
  gtest_main
)
add_test(NAME ${PROJECT_NAME}_icp_gtests 
  COMMAND ${PROJECT_NAME}_icp_gtests
)


add_executable(${PROJECT_NAME}_ndt_tests
  tests/ndt_tests.cpp
)
target_include_directories(${PROJECT_NAME}_ndt_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_ndt_tests
    ${PROJECT_NAME}
    Catch2::Catch2
)

add_executable(${PROJECT_NAME}_gicp_tests
  tests/gicp_tests.cpp
)
target_include_directories(${PROJECT_NAME}_gicp_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_gicp_tests
    ${PROJECT_NAME}
    Catch2::Catch2
)

#[[
add_executable(${PROJECT_NAME}_multi_matcher_tests
  tests/multi_matcher_tests.cpp
)
target_include_directories(${PROJECT_NAME}_multi_matcher_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_multi_matcher_tests
    ${PROJECT_NAME}
    Catch2::Catch2
)
]]

# Copy the test data
file(COPY tests/data tests/config DESTINATION ${PROJECT_BINARY_DIR}/tests)

file(COPY tests/run_all_tests.bash
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)
