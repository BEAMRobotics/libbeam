PROJECT(beam_optimization)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    beam::utils
    beam::calibration
    ceres
  SOURCES
)

################ tests ##################

add_executable(${PROJECT_NAME}_ceres_radtan_tests
  tests/radtan_reprojection_tests.cpp
)
target_include_directories(${PROJECT_NAME}_ceres_radtan_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_ceres_radtan_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)
add_executable(${PROJECT_NAME}_ceres_point_to_line_tests
  tests/point_to_line_tests.cpp
)

target_include_directories(${PROJECT_NAME}_ceres_point_to_line_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_ceres_point_to_line_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)
add_executable(${PROJECT_NAME}_ceres_point_to_plane_tests
  tests/point_to_plane_tests.cpp
)

target_include_directories(${PROJECT_NAME}_ceres_point_to_plane_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_ceres_point_to_plane_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

add_executable(${PROJECT_NAME}_ceres_pose_prior_tests
  tests/pose_prior_tests.cpp
)
target_include_directories(${PROJECT_NAME}_ceres_pose_prior_tests
  PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME}_ceres_pose_prior_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

file(COPY tests/run_all_tests.bash
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)
