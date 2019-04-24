PROJECT(beam_calibration)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    beam::utils
  Boost::boost
    catkin::catkin
    Eigen3::Eigen
    Catch2::Catch2
    nlohmann_json::nlohmann_json
  SOURCES
    src/TfTree.cpp
    src/Intrinsics.cpp
    src/Pinhole.cpp
    #src/Ladybug.cpp
    #src/Fisheye.cpp
)

################ tests ##################
add_executable(${PROJECT_NAME}_intrinsic_tests
  tests/pinhole_test.cpp
)

target_include_directories(${PROJECT_NAME}_intrinsic_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_intrinsic_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

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
#######################################

############# Debug files #############
add_executable(${PROJECT_NAME}_debug
  src/debug.cpp
)
target_include_directories(${PROJECT_NAME}_debug
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_debug
  ${PROJECT_NAME}
  catkin::catkin
)
#######################################
