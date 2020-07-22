PROJECT(beam_calibration)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    beam::utils
    Boost::boost
    Catch2::Catch2
    catkin::catkin
    Eigen3::Eigen
    Ladybug::Ladybug
    nlohmann_json::nlohmann_json
    yaml-cpp
    OpenCV::OpenCV
  SOURCES
    src/TfTree.cpp
    src/Radtan.cpp
    src/DoubleSphere.cpp
    src/CameraModel.cpp
    src/Ladybug.cpp
    src/KannalaBrandt.cpp
)

################ tests ##################
add_executable(${PROJECT_NAME}_extrinsics_tests
  tests/TfTree_test.cpp
)

target_include_directories(${PROJECT_NAME}_extrinsics_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_extrinsics_tests
  ${PROJECT_NAME}
  Catch2::Catch2
  Boost::boost
  catkin::catkin
)

add_executable(${PROJECT_NAME}_double_sphere_tests
  tests/double_sphere_tests.cpp
)

target_include_directories(${PROJECT_NAME}_double_sphere_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_double_sphere_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

add_executable(${PROJECT_NAME}_radtan_tests
  tests/radtan_tests.cpp
)

target_include_directories(${PROJECT_NAME}_radtan_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_radtan_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

add_executable(${PROJECT_NAME}_kannala_brandt_tests
  tests/kannala_brandt_tests.cpp
)

target_include_directories(${PROJECT_NAME}_kannala_brandt_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_kannala_brandt_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

#######################################
