PROJECT(beam_calibration)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    beam::utils
    Boost::boost
    Catch2::Catch2
    catkin::catkin
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
  SOURCES
    src/TfTree.cpp
    src/CameraModel.cpp
    src/DistortionModel.cpp
    src/PinholeCamera.cpp
    src/RadTanDistortion.cpp
    src/TfTree.cpp

)

################ tests ##################
#[[
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

#######################################
]]
############# Debug files #############
add_executable(${PROJECT_NAME}_debug
  src/_debug.cpp
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
