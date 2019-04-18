PROJECT(beam_containers)

BEAM_ADD_MODULE(${PROJECT_NAME}
    DEPENDS
      Eigen3::Eigen
      OpenCV::OpenCV
      PCL::PCL
      beam::utils
      nlohmann_json::nlohmann_json
    SOURCES
      src/includes.cpp
    )
