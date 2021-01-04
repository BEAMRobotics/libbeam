/** @file
 * @ingroup cv
 */

#pragma once

#include <string>
#include <vector>

#include <beam_cv/detectors/Detector.h>
#include <beam_cv/models/SuperPointModel.h>

namespace beam_cv {

/**
 * Wrapper around SuperPointModel which allows the user to only extract
 * keypoints. For information on how it works and a description of all
 * parameter descriptions, see SuperPointModel.h
 */
class SuperPointDetector : public Detector {
public:
  struct Params {
    int max_features{0};
    float conf_threshold{0.01};
    int border{0};
    int nms_dist_threshold{5};
    int grid_size{0};
    bool use_cuda{false};

    void Print() {
      std::cout << "max_features: " << max_features << "\n"
                << "conf_threshold: " << conf_threshold << "\n"
                << "border: " << border << "\n"
                << "nms_dist_threshold: " << nms_dist_threshold << "\n"
                << "grid_size: " << grid_size << "\n"
                << "use_cuda: " << use_cuda << "\n";
    }
  };

  /**
   * @brief Constructor
   * @param model pointer to the superpoint model
   * @param conf_threshold see SuperPointModel.h
   * @param border see SuperPointModel.h
   * @param nms_dist_threshold see SuperPointModel.h
   * @param grid_size see SuperPointModel.h
   * @param use_cuda see SuperPointModel.h
   */
  SuperPointDetector(const std::shared_ptr<SuperPointModel>& model,
                     int max_features = 0, float conf_threshold = 0.01,
                     int border = 0, int nms_dist_threshold = 5,
                     int grid_size = 0, bool use_cuda = false);

  /**
   * @brief Constructor
   * @param model pointer to the superpoint model
   * @param params
   */
  SuperPointDetector(const std::shared_ptr<SuperPointModel>& model,
                     const SuperPointDetector::Params& params);

  /**
   * @brief Default destructor
   */
  ~SuperPointDetector() override = default;

  /**
   * @brief Detects features in an image. This first calls
   * SuperPointModel::Detect, then SuperPointModel::GetKeypoints
   * @param image the image to detect features in.
   * @return a vector containing all of the keypoints found within the image,
   * after filtering.
   */
  std::vector<cv::KeyPoint> DetectFeatures(const cv::Mat& image);

private:
  Params params_;

  /**
   * Shared pointer to the SuperPointModel. This should be shared with the
   * associated SuperPointDetector
   */
  std::shared_ptr<SuperPointModel> model_;
};
} // namespace beam_cv
