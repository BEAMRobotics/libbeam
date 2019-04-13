PROJECT(beam_colorize)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    catkin::catkin
    PCL::PCL
    beam::calibration
    OpenCV::OpenCV
  SOURCES
    src/Colorizer.cpp
    src/Projection.cpp
    src/RayTrace.cpp
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
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
#######################################
