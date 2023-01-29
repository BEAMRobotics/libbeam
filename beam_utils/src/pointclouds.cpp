#include <beam_utils/pointclouds.h>

#include <pcl/common/transforms.h>

#include <beam_utils/math.h>

namespace beam {

sensor_msgs::PointCloud2 PCLToROS(const PointCloud& cloud,
                                  const ros::Time& time,
                                  const std::string& frame_id, uint32_t seq) {
  // Convert to pointcloud2 data type
  pcl::PCLPointCloud2 cloud2;
  pcl::toPCLPointCloud2(cloud, cloud2);

  // Convert to ros msg
  sensor_msgs::PointCloud2 ros_cloud;
  beam::pcl_conversions::fromPCL(cloud2, ros_cloud);

  // update header info
  ros_cloud.header.stamp = time;
  ros_cloud.header.seq = seq;
  ros_cloud.header.frame_id = frame_id;

  return ros_cloud;
}

PointCloud ROSToPCL(const sensor_msgs::PointCloud2& msg, ros::Time& time,
                    std::string& frame_id, uint32_t& seq) {
  // Convert from ROS to pcl pointcloud2
  pcl::PCLPointCloud2 cloud2;
  beam::pcl_conversions::toPCL(msg, cloud2);

  // convert to pcl pointcloud
  PointCloud cloud;
  pcl::fromPCLPointCloud2(cloud2, cloud);

  // get info from header
  time = msg.header.stamp;
  frame_id = msg.header.frame_id;
  seq = msg.header.seq;

  return cloud;
}

void ROSToPCL(pcl::PointCloud<PointXYZIRT>& cloud_out,
              const sensor_msgs::PointCloud2& msg, ros::Time& time,
              std::string& frame_id, uint32_t& seq) {
  // Convert from ROS to pcl pointcloud2
  pcl::PCLPointCloud2 cloud2;
  beam::pcl_conversions::toPCL(msg, cloud2);

  // convert to pcl pointcloud
  PointCloud cloud;
  pcl::fromPCLPointCloud2(cloud2, cloud_out);

  // get info from header
  time = msg.header.stamp;
  frame_id = msg.header.frame_id;
  seq = msg.header.seq;
}

void ROSToPCL(pcl::PointCloud<PointXYZITRRNR>& cloud_out,
              const sensor_msgs::PointCloud2& msg, ros::Time& time,
              std::string& frame_id, uint32_t& seq) {
  // Convert from ROS to pcl pointcloud2
  pcl::PCLPointCloud2 cloud2;
  beam::pcl_conversions::toPCL(msg, cloud2);

  // convert to pcl pointcloud
  PointCloud cloud;
  pcl::fromPCLPointCloud2(cloud2, cloud_out);

  // get info from header
  time = msg.header.stamp;
  frame_id = msg.header.frame_id;
  seq = msg.header.seq;
}

std::vector<geometry_msgs::Vector3> PCLToROSVector(const PointCloud& cloud) {
  std::vector<geometry_msgs::Vector3> cloud_vec;
  for (const pcl::PointXYZ& p : cloud) {
    geometry_msgs::Vector3 point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    cloud_vec.push_back(point);
  }
  return cloud_vec;
}

std::vector<geometry_msgs::Vector3> PCLToROSVector(const PointCloudIRT& cloud) {
  std::vector<geometry_msgs::Vector3> cloud_vec;
  for (const PointXYZIRT& p : cloud) {
    geometry_msgs::Vector3 point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    cloud_vec.push_back(point);
  }
  return cloud_vec;
}

PointCloud ROSVectorToPCL(const std::vector<geometry_msgs::Vector3>& vector) {
  PointCloud cloud;
  for (const geometry_msgs::Vector3& p : vector) {
    pcl::PointXYZ point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    cloud.push_back(point);
  }
  return cloud;
}

PointCloudIRT
    ROSVectorToPCLIRT(const std::vector<geometry_msgs::Vector3>& vector) {
  PointCloudIRT cloud;
  for (const geometry_msgs::Vector3& p : vector) {
    PointXYZIRT point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    cloud.push_back(point);
  }
  return cloud;
}

void PCLPointToPose(const PointXYZIRPYT& point, ros::Time& time,
                    Eigen::Matrix4d& T) {
  T = Eigen::Matrix4d::Identity();
  ros::Time t(point.time);
  time = t;

  Eigen::AngleAxisd rollAngle(point.roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd yawAngle(point.yaw, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitchAngle(point.pitch, Eigen::Vector3d::UnitY());
  Eigen::Quaternion<double> q = rollAngle * pitchAngle * yawAngle;
  T.block(0, 0, 3, 3) = q.matrix();
  T(0, 3) = point.x;
  T(1, 3) = point.y;
  T(2, 3) = point.z;
}

PointCloudCol ColorPointCloud(const PointCloud& cloud, uint8_t r, uint8_t g,
                              uint8_t b) {
  PointCloudCol cloud_col;
  for (uint32_t i = 0; i < cloud.size(); i++) {
    auto& p_in = cloud[i];
    pcl::PointXYZRGB p;
    p.x = p_in.x;
    p.y = p_in.y;
    p.z = p_in.z;
    p.r = r;
    p.g = g;
    p.b = b;
    cloud_col.push_back(p);
  }
  return cloud_col;
}

PointCloudCol ColorPointCloud(const PointCloudIRT& cloud, uint8_t r, uint8_t g,
                              uint8_t b) {
  PointCloudCol cloud_col;
  for (uint32_t i = 0; i < cloud.size(); i++) {
    auto& p_in = cloud[i];
    pcl::PointXYZRGB p;
    p.x = p_in.x;
    p.y = p_in.y;
    p.z = p_in.z;
    p.r = r;
    p.g = g;
    p.b = b;
    cloud_col.push_back(p);
  }
  return cloud_col;
}

PointCloudCol AddFrameToCloud(const PointCloudCol& cloud,
                              const PointCloudCol& frame,
                              const Eigen::Matrix4d& T) {
  PointCloudCol cloud_out;
  if (T.isIdentity()) {
    cloud_out += frame;
  } else {
    pcl::transformPointCloud(frame, cloud_out, T);
  }
  cloud_out += cloud;
  return cloud_out;
}

void MergeFrameToCloud(PointCloudCol& cloud, const PointCloudCol& frame,
                       const Eigen::Matrix4d& T) {
  if (T.isIdentity()) {
    cloud += frame;
  } else {
    PointCloudCol frame_transformed;
    pcl::transformPointCloud(frame, frame_transformed, T);
    cloud += frame_transformed;
  }
}

void MergeFrameToCloud(pcl::PointCloud<pcl::PointXYZRGBL>& cloud,
                       const pcl::PointCloud<pcl::PointXYZRGBL>& frame,
                       const Eigen::Matrix4d& T) {
  if (T.isIdentity()) {
    cloud += frame;
  } else {
    pcl::PointCloud<pcl::PointXYZRGBL> frame_transformed;
    pcl::transformPointCloud(frame, frame_transformed, T);
    cloud += frame_transformed;
  }
}

PointCloud AddFrameToCloud(const PointCloud& cloud, const PointCloud& frame,
                           const Eigen::Matrix4d& T) {
  PointCloud cloud_out;
  if (T.isIdentity()) {
    cloud_out += frame;
  } else {
    pcl::transformPointCloud(frame, cloud_out, T);
  }
  cloud_out += cloud;
  return cloud_out;
}

PointCloudCol AddFrameToCloud(const PointCloudCol& cloud,
                              const Eigen::Matrix4d& T, double increment,
                              double length) {
  PointCloudCol frame = CreateFrameCol(increment, length);
  PointCloudCol cloud_out;
  if (T.isIdentity()) {
    cloud_out += frame;
  } else {
    pcl::transformPointCloud(frame, cloud_out, T);
  }
  cloud_out += cloud;
  return cloud_out;
}

PointCloud AddFrameToCloud(const PointCloud& cloud, const Eigen::Matrix4d& T,
                           double increment, double length) {
  PointCloud frame = CreateFrame(increment, length);
  PointCloud cloud_out;
  if (T.isIdentity()) {
    cloud_out += frame;
  } else {
    pcl::transformPointCloud(frame, cloud_out, T);
  }
  cloud_out += cloud;
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

pcl::PointCloud<pcl::PointXYZL> CreateFrame(const ros::Time& t,
                                            double increment, double length) {
  pcl::PointCloud<pcl::PointXYZL> frame;
  double cur_length{0};
  pcl::PointXYZL pointX(0, 0, 0);
  pcl::PointXYZL pointY(0, 0, 0);
  pcl::PointXYZL pointZ(0, 0, 0);
  pointX.label = t.toSec();
  pointY.label = t.toSec();
  pointZ.label = t.toSec();

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

pcl::PointCloud<pcl::PointXYZRGBL>
    CreateFrameCol(const ros::Time& t, double increment, double length) {
  pcl::PointCloud<pcl::PointXYZRGBL> frame;
  double cur_length{0};
  pcl::PointXYZRGBL pointX(0, 0, 0, 255, 0, 0, t.toSec());
  pcl::PointXYZRGBL pointY(0, 0, 0, 0, 255, 0, t.toSec());
  pcl::PointXYZRGBL pointZ(0, 0, 0, 0, 0, 255, t.toSec());

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

void AddNoiseToCloud(PointCloud& cloud, double max_pert, bool random_seed) {
  if (random_seed) { srand(time(NULL)); }

  for (size_t i = 0; i < cloud.size(); i++) {
    cloud.points.at(i).x += beam::randf(max_pert, -max_pert);
    cloud.points.at(i).y += beam::randf(max_pert, -max_pert);
    cloud.points.at(i).z += beam::randf(max_pert, -max_pert);
  }
}

} // namespace beam
