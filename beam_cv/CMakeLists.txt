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
    src/Morphology.cpp
    src/DepthMap.cpp
)

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
  Eigen3::Eigen
)
