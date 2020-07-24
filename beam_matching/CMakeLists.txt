PROJECT(beam_matching)

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
    src/NdtMatcher.cpp
    src/GicpMatcher.cpp
)

##################### TESTS #########################

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

add_executable(${PROJECT_NAME}_ndt_tests
  tests/ndt_tests.cpp
)

target_include_directories(${PROJECT_NAME}_ndt_tests
  PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME}_ndt_tests
    ${PROJECT_NAME}
    beam::utils
    Eigen3::Eigen
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
    beam::utils
    Eigen3::Eigen
    Catch2::Catch2
)

# Copy the test data
file(COPY tests/data tests/config DESTINATION ${PROJECT_BINARY_DIR}/tests)

file(COPY tests/run_all_tests.bash
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)
