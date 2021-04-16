#include <beam_utils/pointclouds.h>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

namespace beam {

sensor_msgs::PointCloud2 PCLToROS(const PointCloudPtr& cloud,
                                  const ros::Time& time,
                                  const std::string& frame_id, uint32_t seq) {
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

PointCloudColPtr ColorPointCloud(const PointCloudPtr& cloud, uint8_t r,
                                 uint8_t g, uint8_t b) {
  PointCloudColPtr cloud_col = boost::make_shared<PointCloudCol>();
  for (auto it = cloud->begin(); it != cloud->end(); it++) {
    pcl::PointXYZRGB p;
    p.x = it->x;
    p.y = it->y;
    p.z = it->z;
    p.r = r;
    p.g = g;
    p.b = b;
    cloud_col->points.push_back(p);
  }
  return cloud_col;
}

PointCloudColPtr AddFrameToCloud(const PointCloudColPtr& cloud,
                                 const PointCloudColPtr& frame,
                                 const Eigen::Matrix4d& T) {
  PointCloudColPtr cloud_out = boost::make_shared<PointCloudCol>();
  if (T.isIdentity()) {
    *cloud_out += *frame;
  } else {
    pcl::transformPointCloud(*frame, *cloud_out, T);
  }
  *cloud_out += *cloud;
  return cloud_out;
}

PointCloudPtr AddFrameToCloud(const PointCloudPtr& cloud,
                              const PointCloudPtr& frame,
                              const Eigen::Matrix4d& T) {
  PointCloudPtr cloud_out = boost::make_shared<PointCloud>();
  if (T.isIdentity()) {
    *cloud_out += *frame;
  } else {
    pcl::transformPointCloud(*frame, *cloud_out, T);
  }
  *cloud_out += *cloud;
  return cloud_out;
}

PointCloudColPtr AddFrameToCloud(const PointCloudColPtr& cloud,
                                 const Eigen::Matrix4d& T, double increment,
                                 double length) {
  PointCloudColPtr frame = boost::make_shared<PointCloudCol>();
  *frame = CreateFrameCol(increment, length);
  PointCloudColPtr cloud_out = boost::make_shared<PointCloudCol>();
  if (T.isIdentity()) {
    *cloud_out += *frame;
  } else {
    pcl::transformPointCloud(*frame, *cloud_out, T);
  }
  *cloud_out += *cloud;
  return cloud_out;
}

PointCloudPtr AddFrameToCloud(const PointCloudPtr& cloud,
                              const Eigen::Matrix4d& T, double increment,
                              double length) {
  PointCloudPtr frame =
      boost::make_shared<PointCloud>(CreateFrame(increment, length));
  PointCloudPtr cloud_out = boost::make_shared<PointCloud>();
  if (T.isIdentity()) {
    *cloud_out += *frame;
  } else {
    pcl::transformPointCloud(*frame, *cloud_out, T);
  }
  *cloud_out += *cloud;
  return cloud_out;
}

PointCloud CreateFrame(double increment, double length) {
  PointCloud frame;
  double cur_length{0};
  pcl::PointXYZ pointX(0, 0, 0);
  pcl::PointXYZ pointY(0, 0, 0);
  pcl::PointXYZ pointZ(0, 0, 0);
  while (cur_length < length) {
    pointX.x = cur_length;
    pointY.y = cur_length;
    pointZ.z = cur_length;
    frame.push_back(pointX);
    frame.push_back(pointY);
    frame.push_back(pointZ);
    cur_length += increment;
  }
  return frame;
}

PointCloudCol CreateFrameCol(double increment, double length) {
  PointCloudCol frame;
  double cur_length{0};
  pcl::PointXYZRGB pointX;
  pcl::PointXYZRGB pointY;
  pcl::PointXYZRGB pointZ;
  pointX.r = 255;
  pointY.g = 255;
  pointZ.b = 255;
  while (cur_length < length) {
    pointX.x = cur_length;
    pointY.y = cur_length;
    pointZ.z = cur_length;
    frame.push_back(pointX);
    frame.push_back(pointY);
    frame.push_back(pointZ);
    cur_length += increment;
  }
  return frame;
}

} // namespace beam
