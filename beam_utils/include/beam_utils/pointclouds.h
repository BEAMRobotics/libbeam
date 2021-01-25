/** @file
 * @ingroup utils
 *
 * Utility pointcloud functions that do not fit anywhere else.
 *
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

namespace beam {
/** @addtogroup utils
 *  @{ */

#ifndef BEAM_PCL_TYPEDEF
#  define BEAM_PCL_TYPEDEF
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudCol;
typedef PointCloudCol::Ptr PointCloudColPtr;

#endif // BEAM_PCL_TYPEDEF

static ros::Time time_tmp = ros::Time(0);
static std::string string_tmp = "";
static uint32_t seq_tmp = 0;

sensor_msgs::PointCloud2 PCLToROS(const PointCloudPtr& cloud,
                                  const ros::Time& time = ros::Time(0),
                                  const std::string& frame_id = "",
                                  uint32_t seq = 0);

PointCloudPtr ROSToPCL(const sensor_msgs::PointCloud2& msg,
                       ros::Time& time = time_tmp,
                       std::string& frame_id = string_tmp,
                       uint32_t& seq = seq_tmp);

/** @} group utils */
} // namespace beam
