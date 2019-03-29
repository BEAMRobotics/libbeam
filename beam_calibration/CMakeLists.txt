PROJECT(beam_calibration)

BEAM_ADD_MODULE(${PROJECT_NAME}
  DEPENDS
    beam::utils
    catkin::catkin
    Eigen3::Eigen
  SOURCES
    #src/TFTree.cpp
    src/Intrinsics.cpp
    src/Pinhole.cpp
    #src/Ladybug.cpp
    #src/Fisheye.cpp
)
