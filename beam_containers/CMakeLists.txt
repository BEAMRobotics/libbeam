PROJECT(beam_containers)

BEAM_ADD_MODULE(${PROJECT_NAME}
    DEPENDS
      Eigen3::Eigen
      OpenCV::OpenCV
      PCL::PCL
      beam::utils
    SOURCES
      src/includes.cpp
    )
