PROJECT(beam_calibration)

SET(PROJECT_DEPENDS 
    beam::utils
    tf2::tf2
    OpenCV::OpenCV
  )

SET(PROJECT_SOURCES
    src/TfTree.cpp
    src/ConvertCameraModel.cpp
    src/Radtan.cpp
    src/DoubleSphere.cpp
    src/CameraModel.cpp
    src/KannalaBrandt.cpp
    src/Cataditropic.cpp
)  

IF(BUILD_LADYBUG)
   list(APPEND PROJECT_DEPENDS Ladybug::Ladybug)
   list(APPEND PROJECT_SOURCES src/Ladybug.cpp)
ENDIF()


BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    ${PROJECT_DEPENDS}
  SOURCES
    ${PROJECT_SOURCES}
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

add_executable(${PROJECT_NAME}_cataditropic_tests
  tests/cataditropic_tests.cpp
)

target_include_directories(${PROJECT_NAME}_cataditropic_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_cataditropic_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

add_executable(${PROJECT_NAME}_convert_camera_models_tests
  tests/convert_camera_models_tests.cpp
)

target_include_directories(${PROJECT_NAME}_convert_camera_models_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_convert_camera_models_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

IF(BUILD_LADYBUG)
  add_executable(${PROJECT_NAME}_ladybug_tests
    tests/ladybug_tests.cpp
  )

  target_include_directories(${PROJECT_NAME}_ladybug_tests
    PUBLIC
      include
  )
  target_link_libraries(${PROJECT_NAME}_ladybug_tests
    ${PROJECT_NAME}
    Catch2::Catch2
  )
ENDIF()


file(COPY tests/run_all_tests.bash
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

#######################################
