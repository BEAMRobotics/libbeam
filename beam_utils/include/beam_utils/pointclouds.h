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

#ifndef BEAM_PCL_TYPEDEF
#  define BEAM_PCL_TYPEDEF

/**
 * @brief typedefs for commonly used pcl clouds
 */
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudCol;
typedef PointCloudCol::Ptr PointCloudColPtr;

#endif // BEAM_PCL_TYPEDEF

namespace beam {
/** @addtogroup utils
 *  @{ */

static ros::Time time_tmp = ros::Time(0);
static std::string string_tmp = "";
static uint32_t seq_tmp = 0;

/**
 * @brief Convert from a pcl pointcloud to a ROS pointcloud
 * @param cloud pcl pointcloud
 * @param time stamp
 * @param frame_id frame associated with the lidar
 * @param seq scan number
 * @return ros pointcloud
 */
sensor_msgs::PointCloud2 PCLToROS(const PointCloudPtr& cloud,
                                  const ros::Time& time = ros::Time(0),
                                  const std::string& frame_id = "",
                                  uint32_t seq = 0);

/**
 * @brief Convert from a ROS pointcloud to a pcl pointcloud
 * @param cloud ros pointcloud
 * @param time stamp
 * @param frame_id frame associated with the lidar
 * @param seq scan number
 * @return pcl point cloud
 */
PointCloudPtr ROSToPCL(const sensor_msgs::PointCloud2& msg,
                       ros::Time& time = time_tmp,
                       std::string& frame_id = string_tmp,
                       uint32_t& seq = seq_tmp);

/**
 * @brief Add RGB color to a pcl pointcloud
 * @param r red color intensity
 * @param g green color intensity
 * @param b blue color intensity
 * @return colored pointcloud
 */
PointCloudColPtr ColorPointCloud(const PointCloudPtr& cloud, uint8_t r,
                                 uint8_t g, uint8_t b);

/**
 * @brief Add a coordinate frame (built a prebuilt pointcloud) to a color pcl
 * pointcloud
 * @param cloud original cloud
 * @param frame poincloud frame to add
 * @param T optional transform to apply to the frame to get it in the cloud
 * reference frame
 * @return pointcloud containing frame
 */
PointCloudColPtr
    AddFrameToCloud(const PointCloudColPtr& cloud,
                    const PointCloudColPtr& frame,
                    const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity());

/**
 * @brief Add a coordinate frame (built a prebuilt pointcloud) to a pcl
 * pointcloud
 * @param cloud original cloud
 * @param frame poincloud frame to add
 * @param T optional transform to apply to the frame to get it in the cloud
 * reference frame
 * @return pointcloud containing frame
 */
PointCloudPtr
    AddFrameToCloud(const PointCloudPtr& cloud, const PointCloudPtr& frame,
                    const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity());

/**
 * @brief Add a coordinate frame to a color pcl pointcloud. This function calls
 * CreateFrameCol to build the frame. The coordinate frame will be colored RGB
 * for frames XYZ, respectively.
 * @param cloud original cloud
 * @param T optional transform to apply to the frame to get it in the cloud
 * reference frame
 * @param increment distance between points in the frame, in meters
 * @param length frame length in meters
 * @return color pointcloud containing frame
 */
PointCloudColPtr
    AddFrameToCloud(const PointCloudColPtr& cloud,
                    const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity(),
                    double increment = 0.01, double length = 0.3);

/**
 * @brief Add a coordinate frame to a pcl pointcloud. This function calls
 * CreateFrameCol to build the frame
 * @param cloud original cloud
 * @param T optional transform to apply to the frame to get it in the cloud
 * reference frame
 * @param increment distance between points in the frame, in meters
 * @param length frame length in meters
 * @return pointcloud containing frame
 */
PointCloudPtr
    AddFrameToCloud(const PointCloudPtr& cloud,
                    const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity(),
                    double increment = 0.01, double length = 0.3);

/**
 * @brief Build a coordinate frame of points.
 * @param increment distance between points in the frame, in meters
 * @param length frame length in meters
 * @return pointcloud frame
 */
PointCloud CreateFrame(double increment = 0.01, double length = 0.3);

/**
 * @brief Build a coordinate frame of points.  The coordinate frame will be
 * colored RGB for frames XYZ, respectively.
 * @param increment distance between points in the frame, in meters
 * @param length frame length in meters
 * @return colored pointcloud frame
 */
PointCloudCol CreateFrameCol(double increment = 0.01, double length = 0.3);

/** @} group utils */
} // namespace beam
