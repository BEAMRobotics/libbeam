PROJECT(beam_colorize)

find_package(catkin REQUIRED
  COMPONENTS
    sensor_msgs
)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    PCL::PCL
  SOURCES
    src/Colorizer.cpp
    src/Projection.cpp
    src/RayTrace.cpp
)
