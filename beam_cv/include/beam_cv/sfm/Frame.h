/** @file
 * @ingroup cv
 */

#pragma once

#include <beam_utils/utils.h>

namespace beam_cv {

/** Representation of a generic keypoint detector
 */
class Frame {
public:
  /**
   * @brief Default constructor
   */
  Frame() = default;

  /**
   * @brief Custom Constructor
   */
  Frame(cv::Mat image, ros::Time time) {
    static int id = 0;
    id_ = id++;
  }

  /**
   * @brief Default destructor
   */
  virtual ~Frame() = default;

  /**
   * @brief Get pose
   */
  Eigen::Matrix4d GetPose();

  /**
   * @brief Get time
   */
  ros::Time GetPose();

private:
  uint64_t id_;
  cv::Mat image_;
  ros::Time time_;
  Eigen::Matrix4d pose_;
  // Preintegrator preint_;
};

} // namespace beam_cv
