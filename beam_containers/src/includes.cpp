#include "beam_containers/CalibrationContainer.h"
#include "beam_containers/ImageBridge.h"
#include "beam_containers/PointBridge.h"
#include "beam_containers/Utilities.h"

namespace beam_containers {

// Common code will go here

  void  CalibrationContainer::LoadCalibrations(const string& path_to_CalibrationMMDDYYY)
  {
    Camera camera_model;
    std::string tmp_filename;
    int i = 0;
    if(boost::filesystem::is_directory(path_to_CalibrationMMDDYYY)) {
      for (auto & entry : boost::make_iterator_range(boost::filesystem::directory_iterator(path_to_CalibrationMMDDYYY), {}))
      {
        tmp_filename = entry.path().filename().string();
        if (tmp_filename == "extrinsics.json")
        {
          extrinsics_.LoadJSON(entry.path().string());
        }
        else
        {
          camera_model.instrinsics = Create(entry.path().string());
          camera_model.camera_id = i;
          cameras_[i] = camera_model;   
          i++;
        }
      }
    }
    else
    {
       cout << "path_to_CalibrationMMDDYYY: " << path_to_CalibrationMMDDYYY << " is not a directory.\nCalibrations not loaded.\n";
    }
  }

  void  CalibrationContainer::SaveCalibrations(const string& path_to_CalibrationMMDDYYY)
  {
    boost::filesystem::path path_to_CalibrationMMDDYYY_p;
    for (size_t i = 0; i < cameras_.size(); i++)
    {
       path_to_CalibrationMMDDYYY_p = path_to_CalibrationMMDDYYY;
       path_to_CalibrationMMDDYYY_p = path_to_CalibrationMMDDYYY_p / ("F" + boost::lexical_cast<std::string>(i) + ".json");
       cameras_[i].instrinsics.SaveJSON(path_to_CalibrationMMDDYYY_p.string())   
    }
    path_to_CalibrationMMDDYYY_p = path_to_CalibrationMMDDYYY;
    path_to_CalibrationMMDDYYY_p = path_to_CalibrationMMDDYYY_p / "extrinsics.json";
    extrinsics_.SaveJSON(path_to_CalibrationMMDDYYY_p.string());
  }

}
