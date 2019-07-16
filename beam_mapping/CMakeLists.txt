PROJECT(beam_mapping)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    beam::utils
    beam::calibration
    beam::filtering
    Boost::boost
    Catch2::Catch2
    catkin::catkin
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    PCL::PCL
  SOURCES
    src/MapBuilder.cpp
    src/Poses.cpp
)

################ tests ##################

add_executable(${PROJECT_NAME}_map_builder_tests
  tests/MapBuilderTest.cpp
)

target_include_directories(${PROJECT_NAME}_map_builder_tests
  PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME}_map_builder_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

add_executable(${PROJECT_NAME}_poses_test
  tests/PosesTest.cpp
)

target_include_directories(${PROJECT_NAME}_poses_test
  PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME}_poses_test
  ${PROJECT_NAME}
  Catch2::Catch2
)
