PROJECT(beam_utils)

BEAM_ADD_MODULE(${PROJECT_NAME}
    DEPENDS
    Eigen3::Eigen
    yaml-cpp
    SOURCES
    src/config.cpp
    src/math.cpp
    src/time.cpp
    src/angles.cpp
    )
