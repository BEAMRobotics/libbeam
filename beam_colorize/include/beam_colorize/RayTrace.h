/** @file
 * @ingroup colorizer
 */

#pragma once

#include <mutex>
#include <thread>
#include <tuple>
#include <pcl/point_cloud.h>
#include <beam_colorize/Colorizer.h>

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
   * @brief Method for colorizing a point cloud
   * @return Colored point cloud pointer
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColorizePointCloud() const override;

  /**
   * @brief
   * @return
   */
  virtual pcl::PointCloud<beam_containers::PointBridge>::Ptr
      ColorizeMask() const override;

private:
  /**
   * @brief Method for removing unneccessary points in cloud
   * @return Reduced point cloud
   */
  std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, std::vector<int>>
      ReduceCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
                  std::shared_ptr<beam_calibration::CameraModel>) const;

protected:
  double hit_threshold_ = 0.01;
};

/** @} group colorizer */

} // namespace beam_colorize
