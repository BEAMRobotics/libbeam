/** @file
 * @ingroup calibration
 */

#pragma once
// beam
#include "beam_calibration/include/Refactor/CameraModel.h"
#include "beam_calibration/include/Refactor/EquidistantDistortion.h"
#include "beam_calibration/include/Refactor/LadybugCamera.h"
#include "beam_calibration/include/Refactor/PinholeCamera.h"
#include "beam_calibration/include/Refactor/RadtanDistortion.h"

// file io
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

namespace beam_calibration {

class CameraFactory {
public:
  /**
   * @brief Default constructor
   */
  CameraFactory() = default;

  /**
   * @brief Default destructor
   */
  virtual ~CameraFactory() = default;

  static std::shared_ptr<CameraModel>
      Create(CameraType type, std::string& file_location, uint cam_id = 0);

  static std::shared_ptr<CameraModel> LoadJSON(std::string& file_location);

  static std::shared_ptr<CameraModel> LoadYAML(std::string& file_location);

} // namespace beam_calibration