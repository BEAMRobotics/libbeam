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
typedef std::shared_ptr<PointCloud> PointCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudCol;
typedef std::shared_ptr<PointCloudCol> PointCloudColPtr;

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
PointCloudCol ColorPointCloud(const PointCloud& cloud, uint8_t r, uint8_t g,
                              uint8_t b);

/**
 * @brief Add a coordinate frame (built a prebuilt pointcloud) to a color pcl
 * pointcloud
 * @param cloud original cloud
 * @param frame poincloud frame to add
 * @param T optional transform to apply to the frame to get it in the cloud
 * reference frame
 * @return pointcloud containing frame
 */
PointCloudCol
    AddFrameToCloud(const PointCloudCol& cloud, const PointCloudCol& frame,
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
PointCloud
    AddFrameToCloud(const PointCloud& cloud, const PointCloud& frame,
                    const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity());

/**
 * @brief Merge a coordinate frame (built a prebuilt pointcloud) to a pcl
 * pointcloud
 * @param cloud reference to original cloud
 * @param frame poincloud frame to add
 * @param T optional transform to apply to the frame to get it in the cloud
 * reference frame
 * @return pointcloud containing frame
 */
void MergeFrameToCloud(PointCloudCol& cloud, const PointCloudCol& frame,
                       const Eigen::Matrix4d& T);

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
PointCloudCol
    AddFrameToCloud(const PointCloudCol& cloud,
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
PointCloud
    AddFrameToCloud(const PointCloud& cloud,
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
 * @brief Build a coordinate frame of points. The coordinate frame will be
 * colored RGB for frames XYZ, respectively.
 * @param increment distance between points in the frame, in meters
 * @param length frame length in meters
 * @return colored pointcloud frame
 */
PointCloudCol CreateFrameCol(double increment = 0.01, double length = 0.3);

/**
 * @brief Build a coordinate frame of points with timestamp as a label.
 * @param t timestamp to add to the point labels
 * @param increment distance between points in the frame, in meters
 * @param length frame length in meters
 * @return pointcloud frame
 */
pcl::PointCloud<pcl::PointXYZL> CreateFrame(const ros::Time& t,
                                            double increment = 0.01,
                                            double length = 0.3);

/**
 * @brief Build a coordinate frame of points with timestamp as a label. The
 * coordinate frame will be colored RGB for frames XYZ, respectively.
 * @param t timestamp to add to the point labels
 * @param increment distance between points in the frame, in meters
 * @param length frame length in meters
 * @return colored pointcloud frame
 */
pcl::PointCloud<pcl::PointXYZRGBL> CreateFrameCol(const ros::Time& t,
                                                  double increment = 0.01,
                                                  double length = 0.3);

/**
 * @brief Calculate the absolute distance of the point to the origin.
 * @param p The point.
 * @return The distance to the point.
 */
template <typename PointT>
inline float PointDistance(const PointT& p) {
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

/**
 * @brief Calculate the squared distance of the point to the origin.
 * @param p The point.
 * @return The squared distance to the point.
 */
template <typename PointT>
inline float SquaredPointDistance(const PointT& p) {
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
inline float SquaredDiff(const PointT& a, const PointT& b, const float& wb) {
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
inline float SquaredDiff(const PointT& a, const PointT& b) {
  float diffX = a.x - b.x;
  float diffY = a.y - b.y;
  float diffZ = a.z - b.z;

  return diffX * diffX + diffY * diffY + diffZ * diffZ;
}

template <typename PointT>
inline void getMinMax3D(const pcl::PointCloud<PointT>& cloud, PointT& min_pt,
                        PointT& max_pt) {
  Eigen::Array4f min_p, max_p;
  min_p.setConstant(FLT_MAX);
  max_p.setConstant(-FLT_MAX);

  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense) {
    for (size_t i = 0; i < cloud.points.size(); ++i) {
      pcl::Array4fMapConst pt = cloud.points[i].getArray4fMap();
      min_p = min_p.min(pt);
      max_p = max_p.max(pt);
    }
  }
  // NaN or Inf values could exist => check for them
  else {
    for (size_t i = 0; i < cloud.points.size(); ++i) {
      // Check if the point is invalid
      if (!std::isfinite(cloud.points[i].x) ||
          !std::isfinite(cloud.points[i].y) || !std::isfinite(cloud.points[i].z))
        continue;
      pcl::Array4fMapConst pt = cloud.points[i].getArray4fMap();
      min_p = min_p.min(pt);
      max_p = max_p.max(pt);
    }
  }
  min_pt.x = min_p[0];
  min_pt.y = min_p[1];
  min_pt.z = min_p[2];
  max_pt.x = max_p[0];
  max_pt.y = max_p[1];
  max_pt.z = max_p[2];
}

/** @} group utils */
} // namespace beam
