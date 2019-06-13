PROJECT(beam_containers)

BEAM_ADD_MODULE(${PROJECT_NAME}
    DEPENDS
      Eigen3::Eigen
      OpenCV::OpenCV
      PCL::PCL
      beam::utils
      nlohmann_json::nlohmann_json
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
