PROJECT(beam_colorize)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    catkin::catkin
    PCL::PCL
    beam::calibration
    beam::containers
    OpenCV::OpenCV
  SOURCES
    src/Colorizer.cpp
    src/Projection.cpp
    src/RayTrace.cpp
)

############# Debug files #############
add_executable(${PROJECT_NAME}_example
  src/example.cpp
)
target_include_directories(${PROJECT_NAME}_example
  PUBLIC
    include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_example
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
#######################################
