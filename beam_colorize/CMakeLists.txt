PROJECT(beam_colorize)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    catkin::catkin
    PCL::PCL
  SOURCES
    src/Colorizer.cpp
    src/Projection.cpp
    src/RayTrace.cpp
)
