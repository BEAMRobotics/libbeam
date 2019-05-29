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
    src/CameraModel.cpp
    src/LadybugCamera.cpp
    src/PinholeCamera.cpp

)

################ tests ##################
#
add_executable(${PROJECT_NAME}_camera_tests
  tests/Camera_test.cpp
)

target_include_directories(${PROJECT_NAME}_camera_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_camera_tests
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
