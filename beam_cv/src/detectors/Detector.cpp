#include <beam_cv/detectors/Detectors.h>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

#include <beam_utils/log.h>

namespace beam_cv {

Detector::Detector(int grid_cols, int grid_rows)
    : grid_cols_(grid_cols), grid_rows_(grid_rows) {}

std::shared_ptr<Detector> Detector::Create(DetectorType type,
                                           const std::string& file_path) {
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

std::vector<cv::KeyPoint> Detector::DetectFeatures(const cv::Mat& image) {
  std::vector<cv::KeyPoint> local_keypoints;
  std::vector<cv::KeyPoint> global_keypoints;

  int grid_width = image.cols / grid_cols_;
  int grid_height = image.rows / grid_rows_;

  for (int y = 0; y + grid_height <= image.rows; y += grid_height) {
    for (int x = 0; x + grid_width <= image.cols; x += grid_width) {
      cv::Rect roi = cv::Rect(x, y, grid_width, grid_height);
      cv::Mat grid = image(roi);
      local_keypoints = DetectLocalFeatures(grid);

      for (size_t i = 0; i < local_keypoints.size(); i++) {
        // use iterator position to translate local global coords
        cv::KeyPoint global_pt = local_keypoints[i];
        global_pt.pt.x += x;
        global_pt.pt.y += y;
        global_keypoints.push_back(global_pt);
      }
    }
  }
  return global_keypoints;
}

} // namespace beam_cv
