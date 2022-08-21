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
   * @brief Method for colorizing a point cloud
   * @param return_in_cam_frame set to false to return the cloud in its original
   * frame. Internally, when a cloud is added it's converted to the camera
   * frame, so here when we return the pointcloud it can be in either frame.
   * @return Colored point cloud pointer
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      ColorizePointCloud(bool return_in_cam_frame = false) const override;

  /**
   * @brief see Colorizer.h
   */
  virtual pcl::PointCloud<beam_containers::PointBridge>::Ptr
      ColorizeMask(bool return_in_cam_frame = false) const override;

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
