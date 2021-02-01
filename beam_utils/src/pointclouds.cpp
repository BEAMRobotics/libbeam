#include <beam_utils/pointclouds.h>

#include <pcl_conversions/pcl_conversions.h>

namespace beam {

sensor_msgs::PointCloud2 PCLToROS(const PointCloudPtr& cloud,
                                  const ros::Time& time, std::string& frame_id,
                                  uint32_t seq) {
  // Convert to pointcloud2 data type
  pcl::PCLPointCloud2 cloud2;
  pcl::toPCLPointCloud2(*cloud, cloud2);

  // Convert to ros msg
  sensor_msgs::PointCloud2 ros_cloud;
  pcl_conversions::fromPCL(cloud2, ros_cloud);

  // update header info
  ros_cloud.header.stamp = time;
  ros_cloud.header.seq = seq;
  ros_cloud.header.frame_id = frame_id;

  return ros_cloud;
}

PointCloudPtr ROSToPCL(const sensor_msgs::PointCloud2& msg, ros::Time& time,
                       std::string& frame_id, uint32_t& seq) {
  // Convert from ROS to pcl pointcloud2
  pcl::PCLPointCloud2 cloud2;
  pcl_conversions::toPCL(msg, cloud2);

  // convert to pcl pointcloud
  PointCloudPtr cloud = boost::make_shared<PointCloud>();
  pcl::fromPCLPointCloud2(cloud2, *cloud);

  // get info from header
  time = msg.header.stamp;
  frame_id = msg.header.frame_id;
  seq = msg.header.seq;

  return cloud;
}

} // namespace beam
