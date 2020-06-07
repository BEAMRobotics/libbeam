PROJECT(beam_cv)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    catkin::catkin
    PCL::PCL
    beam::calibration
    beam::containers
    beam::utils
    OpenCV::OpenCV
    Eigen3::Eigen
  SOURCES
    src/Utils.cpp
    src/DepthMap.cpp
    src/RayCast.cpp
    src/LinearRegression.cpp
    src/DepthCompletion.cpp
)

############# Test files #############
add_executable(${PROJECT_NAME}_utils_tests
  tests/utils_test.cpp
)

target_include_directories(${PROJECT_NAME}_utils_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_utils_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

add_executable(${PROJECT_NAME}_depth_tests
  tests/depthmap_test.cpp
)

target_include_directories(${PROJECT_NAME}_depth_tests
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_depth_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)

add_executable(${PROJECT_NAME}_kitti
  tests/kitti.cpp
)
target_include_directories(${PROJECT_NAME}_kitti
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_kitti
  ${PROJECT_NAME}
  Eigen3::Eigen
  ${CERES_LIBRARIES}
)
