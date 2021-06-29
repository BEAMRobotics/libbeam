#include <beam_cv/detectors/Detectors.h>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

#include <beam_utils/log.h>

namespace beam_cv {

std::shared_ptr<Detector>
    Detector::Create(DetectorType type, const std::string& file_path) {
  // check input file path
  std::string file_to_read = file_path;
  if (!boost::filesystem::exists(file_to_read) && !file_path.empty()) {
    BEAM_WARN("Invalid file path for detector, using default params. Input: {}",
              file_path);
    file_to_read = "";
  }

  // create detector
  if (type == DetectorType::ORB) {
    ORBDetector::Params params;
    params.LoadFromJson(file_to_read);
    return std::make_shared<ORBDetector>(params);
  } else if (type == DetectorType::SIFT) {
    SIFTDetector::Params params;
    params.LoadFromJson(file_to_read);
    return std::make_shared<SIFTDetector>(params);
  } else if (type == DetectorType::FAST) {
    FASTDetector::Params params;
    params.LoadFromJson(file_to_read);
    return std::make_shared<FASTDetector>(params);
  } else if (type == DetectorType::GFTT) {
    GFTTDetector::Params params;
    params.LoadFromJson(file_to_read);
    return std::make_shared<GFTTDetector>(params);
  } else {
    BEAM_WARN("Input detector type not implemented. Using default.");
    ORBDetector::Params params;
    params.LoadFromJson(file_to_read);
    return std::make_shared<ORBDetector>(params);
  }
}

} // namespace beam_cv
