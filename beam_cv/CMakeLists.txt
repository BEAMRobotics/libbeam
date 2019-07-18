PROJECT(beam_cv)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    catkin::catkin
    PCL::PCL
    beam::calibration
    beam::containers
    OpenCV::OpenCV
  SOURCES
    src/Morphology.cpp
)

############# Debug files #############
add_executable(${PROJECT_NAME}_debug
  src/debug.cpp
)
target_include_directories(${PROJECT_NAME}_debug
  PUBLIC
    include
)
