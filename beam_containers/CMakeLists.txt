PROJECT(beam_containers)

BEAM_ADD_MODULE(${PROJECT_NAME}
    DEPENDS
      beam::utils
      beam::cv
    SOURCES
      src/includes.cpp
    )

################ tests ##################

add_executable(${PROJECT_NAME}_image_container_tests
  tests/image_container_tests.cpp
)

target_include_directories(${PROJECT_NAME}_image_container_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_image_container_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

add_executable(${PROJECT_NAME}_point_container_tests
  tests/point_container_tests.cpp
)

target_include_directories(${PROJECT_NAME}_point_container_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_point_container_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

add_executable(${PROJECT_NAME}_landmark_container_tests 
  tests/landmark_container_tests.cpp
)
target_link_libraries(${PROJECT_NAME}_landmark_container_tests 
  ${PROJECT_NAME}
  gtest_main
)
add_test(NAME ${PROJECT_NAME}_landmark_container_tests 
  COMMAND ${PROJECT_NAME}_landmark_container_tests
)

file(COPY tests/run_all_tests.bash
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)
