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
                     int max_features = 0, float conf_threshold = 0.1,
                     int border = 0, int nms_dist_threshold = 0,
                     int grid_size = 0, bool use_cuda = false);

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
  float conf_threshold_;
  int border_;
  int nms_dist_threshold_;
  int max_features_;
  int grid_size_;
  bool use_cuda_;

  /**
   * Shared pointer to the SuperPointModel. This should be shared with the
   * associated SuperPointDetector
   */
  std::shared_ptr<SuperPointModel> model_;
};
} // namespace beam_cv
