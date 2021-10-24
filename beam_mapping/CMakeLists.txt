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
    src/Utils.cpp
)

################ Executables ###################
add_executable(${PROJECT_NAME}_build_map
  src/build_map.cpp
)

target_include_directories(${PROJECT_NAME}_build_map
  PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME}_build_map
  ${PROJECT_NAME}
  beam::utils
  beam::filtering
  beam::calibration
)

add_executable(${PROJECT_NAME}_loop_closed_paths_to_poses
  src/loop_closed_paths_to_poses.cpp
)

target_include_directories(${PROJECT_NAME}_loop_closed_paths_to_poses
  PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME}_loop_closed_paths_to_poses
  ${PROJECT_NAME}
  beam::utils
  beam::filtering
  beam::calibration
)

add_executable(${PROJECT_NAME}_path_messages_to_poses_files
  src/path_messages_to_pose_files.cpp
)

target_include_directories(${PROJECT_NAME}_path_messages_to_poses_files
  PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME}_path_messages_to_poses_files
  ${PROJECT_NAME}
  beam::utils
  beam::filtering
  beam::calibration
)

add_executable(${PROJECT_NAME}_bag_to_poses_file
  src/bag_to_poses_file.cpp
)

target_include_directories(${PROJECT_NAME}_bag_to_poses_file
  PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME}_bag_to_poses_file
  ${PROJECT_NAME}
  beam::utils
  beam::filtering
  beam::calibration
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

file(COPY tests/run_all_tests.bash
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)
