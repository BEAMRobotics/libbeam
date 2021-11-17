PROJECT(beam_colorize)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    beam::calibration
    beam::containers
    beam::cv
  SOURCES
    src/Colorizer.cpp
    src/Projection.cpp
    src/RayTrace.cpp
)
################ tests ##################
add_executable(${PROJECT_NAME}_tests
  tests/colorize_test.cpp
)
target_include_directories(${PROJECT_NAME}_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)
############# Debug files #############
add_executable(${PROJECT_NAME}_demo
  src/colorizer_demo.cpp
)
target_include_directories(${PROJECT_NAME}_demo
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_demo
  ${PROJECT_NAME}
)

file(COPY tests/run_all_tests.bash
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)
#######################################
