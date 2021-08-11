#pragma once

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include <beam_utils/math.h>
#include <beam_utils/time.h>

namespace beam_containers {
/** @addtogroup containers
 *  @{ */

/** Storage type for a landmark measurement as a 2D position.
 *
 * Presumably the landmark measurement has been extracted from a camera image,
 * but it is now a separate entity with no record of which camera image it came
 * from.
 *
 * This class represents a visual landmark, and assigns a unique id to each so
 * it can be tracked across various frames
 *
 * The measurement value is 2D vector, representing the pixel position of the
 * landmark (u, v).
 *
 * @tparam SensorIdType The enum or integral type used to identify camera
 * sensors.
 */
struct LandmarkMeasurement {
  ros::Time time_point;
  uint8_t sensor_id;
  uint64_t landmark_id;
  uint64_t image;
  Eigen::Vector2d value;
  cv::Mat descriptor;

  /**
   * @brief constructor that takes in all values of the measurement
   */
  LandmarkMeasurement(const ros::Time& t, const uint8_t& s, const uint64_t& id,
                      const uint64_t& img, const Eigen::Vector2d& v,
                      const cv::Mat& desc)
      : time_point{t}, sensor_id{s}, landmark_id{id}, image{img}, value{v} {
    descriptor = desc.clone();
  }

  /**
   * @brief constructor that takes in a JSON. JSON must be of the same format
   * that is output by ToJson() function
   */
  LandmarkMeasurement(const nlohmann::json& J) {
    time_point.fromNSec(J["timepoint"]);
    sensor_id = J["sensor_id"];
    landmark_id = J["landmark_id"];
    image = J["image"];
    value[0] = J["value_u"];
    value[1] = J["value_v"];
    std::vector<uint8_t> desc_vec = J["descriptor"];
    descriptor = cv::Mat(1, desc_vec.size(), CV_8U, desc_vec.data()).clone();
    return;
  }

  static ros::Time MinTime() { return ros::TIME_MIN; }

  static ros::Time MaxTime() { return ros::TIME_MAX; }

  nlohmann::json ToJson() const {
    nlohmann::json J;
    J["timepoint"] = time_point.toNSec();
    J["sensor_id"] = sensor_id;
    J["landmark_id"] = landmark_id;
    J["image"] = image;
    J["value_u"] = value[0];
    J["value_v"] = value[1];

    std::vector<int> desc_vec;
    for (int k = 0; k < descriptor.cols; k++) {
      uint8_t cur_value = descriptor.at<uint8_t>(0, k);
      desc_vec.push_back(static_cast<int>(cur_value));
    }
    J["descriptor"] = desc_vec;
    return J;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace beam_containers
