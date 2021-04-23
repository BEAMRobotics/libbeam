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

sensor_msgs::PointCloud2 PCLToROS(const PointCloudPtr& cloud,
                                  const ros::Time& time = ros::Time(0),
                                  const std::string& frame_id = "",
                                  uint32_t seq = 0);

PointCloudPtr ROSToPCL(const sensor_msgs::PointCloud2& msg,
                       ros::Time& time = time_tmp,
                       std::string& frame_id = string_tmp,
                       uint32_t& seq = seq_tmp);

PointCloudColPtr ColorPointCloud(const PointCloudPtr& cloud, uint8_t r,
                                 uint8_t g, uint8_t b);

PointCloudColPtr ColorPointCloud(const PointCloudPtr& cloud, uint8_t r,
                                 uint8_t g, uint8_t b);

PointCloudColPtr
    AddFrameToCloud(const PointCloudColPtr& cloud,
                    const PointCloudColPtr& frame,
                    const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity());

PointCloudPtr
    AddFrameToCloud(const PointCloudPtr& cloud, const PointCloudPtr& frame,
                    const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity());

PointCloudColPtr
    AddFrameToCloud(const PointCloudColPtr& cloud,
                    const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity(),
                    double increment = 0.01, double length = 0.3);

PointCloudPtr
    AddFrameToCloud(const PointCloudPtr& cloud,
                    const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity(),
                    double increment = 0.01, double length = 0.3);

PointCloud CreateFrame(double increment = 0.01, double length = 0.3);

PointCloudCol CreateFrameCol(double increment = 0.01, double length = 0.3);

/** 
 * @brief Calculate the absolute distance of the point to the origin.
 * @param p The point.
 * @return The distance to the point.
 */
template <typename PointT>
inline float PointDistance(const PointT& p)
{
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

/** 
 * @brief Calculate the squared distance of the point to the origin.
 * @param p The point.
 * @return The squared distance to the point.
 */
template <typename PointT>
inline float SquaredPointDistance(const PointT& p)
{
  return p.x * p.x + p.y * p.y + p.z * p.z;
}

/** 
 * @brief Calculate the squared difference of the given two points.
 * @param a The first point.
 * @param b The second point.
 * @param wb The weighting factor for the SECOND point.
 * @return The squared difference between point a and b.
 */
template <typename PointT>
inline float SquaredDiff(const PointT& a, const PointT& b, const float& wb)
{
  float diffX = a.x - b.x * wb;
  float diffY = a.y - b.y * wb;
  float diffZ = a.z - b.z * wb;

  return diffX * diffX + diffY * diffY + diffZ * diffZ;
}

/** 
 * @brief Calculate the squared difference of the given two points.
 * @param a The first point.
 * @param b The second point.
 * @return The squared difference between point a and b.
 */
template <typename PointT>
inline float SquaredDiff(const PointT& a, const PointT& b)
{
  float diffX = a.x - b.x;
  float diffY = a.y - b.y;
  float diffZ = a.z - b.z;

  return diffX * diffX + diffY * diffY + diffZ * diffZ;
}

/** @} group utils */
} // namespace beam
