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
  SOURCES
  #src/Fisheye.cpp
    src/Intrinsics.cpp
  src/Ladybug.cpp
    src/Pinhole.cpp
  src/TfTree.cpp
  )

#[[################ tests ##################
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
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)]]
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
  ${catkin_LIBRARIES}
)
#######################################

############# Debug files #############
add_executable(${PROJECT_NAME}_ladybug_debug
  src/Ladybug_main.cpp
  )
target_include_directories(${PROJECT_NAME}_ladybug_debug
  PUBLIC
  include
  )
target_link_libraries(${PROJECT_NAME}_ladybug_debug
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )
#######################################