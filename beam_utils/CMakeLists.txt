PROJECT(beam_utils)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    Eigen3::Eigen
    catkin::catkin
    yaml-cpp
    PCL::PCL
    OpenCV::OpenCV
  SOURCES
    src/math.cpp
    src/time.cpp
    src/angles.cpp
    src/uf.cpp
)
