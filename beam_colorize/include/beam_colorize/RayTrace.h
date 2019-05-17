/** @file
 * @ingroup colorizer
 */

#pragma once
#include "beam_colorize/Colorizer.h"

// PCL
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

// Standard lib
#include <mutex>
#include <thread>
#include <tuple>

namespace beam_colorize {

/** @addtogroup colorizer
 *  @{ */

/**
 * @brief Class which implements Colorizer interface and provides colorization
 * functionality using ray tracing
 */
class RayTrace : public Colorizer {
public:
  RayTrace();

  ~RayTrace() = default;

  /**
   * @brief Pixel type for iterating through the image
   */
  typedef cv::Point3_<uchar> Pixel;

  /**
   * @brief Method for colorizing a point cloud
   * @return Colored point cloud pointer
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColorizePointCloud() const override;

private:
  uint16_t dilation_;
  uint16_t max_ray_;
  double hit_threshold_;
  /**
   * @brief Method for removing unneccessary points in cloud
   * @return Reduced point cloud
   */
  std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, std::vector<int>>
      ReduceCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
                  std::shared_ptr<cv::Mat>,
                  beam_calibration::Intrinsics*) const;
};

/** @} group colorizer */

} // namespace beam_colorize
