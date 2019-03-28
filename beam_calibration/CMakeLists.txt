PROJECT(beam_calibration)

BEAM_ADD_MODULE(${PROJECT_NAME}
    DEPENDS
    Eigen3::Eigen
    beam::utils
    SOURCES
    #src/TFTree.cpp
    src/Intrinsics.cpp
    src/Pinhole.cpp
    #src/Ladybug.cpp
    #src/Fisheye.cpp
    )
