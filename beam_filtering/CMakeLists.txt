PROJECT(beam_filtering)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    beam::utils
    Boost::boost
    Catch2::Catch2
    catkin::catkin
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    PCL::PCL
  SOURCES
    src/DROR.cpp
    src/CropBox.cpp
)

################ tests ##################

add_executable(${PROJECT_NAME}_dror_tests
  tests/DRORTest.cpp
)

target_include_directories(${PROJECT_NAME}_dror_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_dror_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

add_executable(${PROJECT_NAME}_cropbox_tests
  tests/CropBoxTest.cpp
)

target_include_directories(${PROJECT_NAME}_cropbox_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_cropbox_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

file(COPY tests/run_all_tests.bash
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)
