/** @file
 * @ingroup cv
 */

#pragma once

#include <beam_calibration/CameraModel.h>
#include <beam_calibration/ConvertCameraModel.h>
#include <beam_cv/sfm/Frame.h>
#include <beam_cv/tracker/Tracker.h>
#include <beam_utils/utils.h>

namespace beam_cv {

/** Representation of a generic keypoint detector
 */
class SlidingWindow {
public:
  /**
   * @brief Default constructor
   */
  SlidingWindow() = default;

  /**
   * @brief Custom Constructor
   */
  SlidingWindow(beam_cv::Tracker tracker,
                std::shared_ptr<beam_calibration::CameraModel> cam_model,
                int window_size = 0);

  /**
   * @brief Default destructor
   */
  virtual ~SlidingWindow() = default;

  /**
   * @brief Add Frame to window
   */
  void AddFrame(std::unique_ptr<beam_cv::Frame> frame);

  /**
   * @brief Attempt sfm initialization on current frames
   */
  bool PerformSFM() {
    /*
    1. loop through all current frames and find a pair that satisfies the common
    track and parallax threshold
    2. compute homography between the pair (use opencv, and also determine if pure rotation)
    3. if its not pure rotation, add the decomposed poses to the candidates
        (check slamtools for how to check if pure rotation)
    4. if it is pure rotation then fail
    5. compute essential matrix (use opencv, since images are rectified) and get candidate poses
    8. go through candidate poses and find best one (most inliers)
    9. store the triangulated landmarks
    10. add these poses to the pair of frames (identity + relative pose)
    11. go through every other frame, find 3d correspondences to get a pose from pnp
    12. run a bundle adjustment on the entire window
    */
  }

  /**
   * @brief Get the currently stored frames
   */
  std::deque<std::unique_ptr<beam_cv::Frame>> GetFrames();

private:
  std::shared_ptr<beam_cv::Tracker> tracker_;
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  std::shared_ptr<beam_calibration::UndistortImages> rectifier_;
  std::deque<std::unique_ptr<beam_cv::Frame>> frames_;
  std::map<uint64_t, Eigen::Vector3d> landmark_positions_;
};

} // namespace beam_cv
