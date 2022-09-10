/** @file
 * @ingroup colorizer
 */

#pragma once

#include <beam_colorize/Colorizer.h>
#include <mutex>
#include <pcl/point_cloud.h>
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
   * @brief see Colorizer.h
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColorizePointCloud() const override;

  /**
   * @brief see Colorizer.h
   */
  virtual pcl::PointCloud<beam_containers::PointBridge>::Ptr
      ColorizeMask() const override;

private:
  /**
   * @brief Method for removing unneccessary points in cloud
   * @return Reduced point cloud
   */
  std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, std::vector<int>>
      ReduceCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input) const;

protected:
  double hit_threshold_ = 0.01;
};

/** @} group colorizer */

} // namespace beam_colorize
