/** @file
 * @ingroup utils
 *
 * this is a copy of the pcl_conversions from
 * https://github.com/ros-perception/perception_pcl/blob/melodic-devel/pcl_conversions/include/pcl_conversions/pcl_conversions.h
 *
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/conversions.h>
#include <pcl/PCLHeader.h>
#include <std_msgs/Header.h>
#include <pcl/PCLPointField.h>
#include <sensor_msgs/PointField.h>
#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>

namespace beam {
/** @addtogroup utils
 *  @{ */

namespace pcl_conversions {

/** PCLHeader <=> Header **/
inline void fromPCL(const std::uint64_t& pcl_stamp, ros::Time& stamp) {
  stamp.fromNSec(pcl_stamp * 1000ull); // Convert from us to ns
}

inline void toPCL(const ros::Time& stamp, std::uint64_t& pcl_stamp) {
  pcl_stamp = stamp.toNSec() / 1000ull; // Convert from ns to us
}

inline ros::Time fromPCL(const std::uint64_t& pcl_stamp) {
  ros::Time stamp;
  fromPCL(pcl_stamp, stamp);
  return stamp;
}

inline std::uint64_t toPCL(const ros::Time& stamp) {
  std::uint64_t pcl_stamp;
  toPCL(stamp, pcl_stamp);
  return pcl_stamp;
}

/** PCLHeader <=> Header **/

inline void fromPCL(const pcl::PCLHeader& pcl_header,
                    std_msgs::Header& header) {
  fromPCL(pcl_header.stamp, header.stamp);
  header.seq = pcl_header.seq;
  header.frame_id = pcl_header.frame_id;
}

inline void toPCL(const std_msgs::Header& header, pcl::PCLHeader& pcl_header) {
  toPCL(header.stamp, pcl_header.stamp);
  pcl_header.seq = header.seq;
  pcl_header.frame_id = header.frame_id;
}

inline std_msgs::Header fromPCL(const pcl::PCLHeader& pcl_header) {
  std_msgs::Header header;
  fromPCL(pcl_header, header);
  return header;
}

inline pcl::PCLHeader toPCL(const std_msgs::Header& header) {
  pcl::PCLHeader pcl_header;
  toPCL(header, pcl_header);
  return pcl_header;
}

/** PCLPointField <=> PointField **/

inline void fromPCL(const pcl::PCLPointField& pcl_pf,
                    sensor_msgs::PointField& pf) {
  pf.name = pcl_pf.name;
  pf.offset = pcl_pf.offset;
  pf.datatype = pcl_pf.datatype;
  pf.count = pcl_pf.count;
}

inline void fromPCL(const std::vector<pcl::PCLPointField>& pcl_pfs,
                    std::vector<sensor_msgs::PointField>& pfs) {
  pfs.resize(pcl_pfs.size());
  std::vector<pcl::PCLPointField>::const_iterator it = pcl_pfs.begin();
  int i = 0;
  for (; it != pcl_pfs.end(); ++it, ++i) { fromPCL(*(it), pfs[i]); }
}

inline void toPCL(const sensor_msgs::PointField& pf,
                  pcl::PCLPointField& pcl_pf) {
  pcl_pf.name = pf.name;
  pcl_pf.offset = pf.offset;
  pcl_pf.datatype = pf.datatype;
  pcl_pf.count = pf.count;
}

inline void toPCL(const std::vector<sensor_msgs::PointField>& pfs,
                  std::vector<pcl::PCLPointField>& pcl_pfs) {
  pcl_pfs.resize(pfs.size());
  std::vector<sensor_msgs::PointField>::const_iterator it = pfs.begin();
  int i = 0;
  for (; it != pfs.end(); ++it, ++i) { toPCL(*(it), pcl_pfs[i]); }
}

/** PCLPointCloud2 <=> PointCloud2 **/

inline void copyPCLPointCloud2MetaData(const pcl::PCLPointCloud2& pcl_pc2,
                                       sensor_msgs::PointCloud2& pc2) {
  fromPCL(pcl_pc2.header, pc2.header);
  pc2.height = pcl_pc2.height;
  pc2.width = pcl_pc2.width;
  fromPCL(pcl_pc2.fields, pc2.fields);
  pc2.is_bigendian = pcl_pc2.is_bigendian;
  pc2.point_step = pcl_pc2.point_step;
  pc2.row_step = pcl_pc2.row_step;
  pc2.is_dense = pcl_pc2.is_dense;
}

inline void fromPCL(const pcl::PCLPointCloud2& pcl_pc2,
                    sensor_msgs::PointCloud2& pc2) {
  copyPCLPointCloud2MetaData(pcl_pc2, pc2);
  pc2.data = pcl_pc2.data;
}

inline void moveFromPCL(pcl::PCLPointCloud2& pcl_pc2,
                        sensor_msgs::PointCloud2& pc2) {
  copyPCLPointCloud2MetaData(pcl_pc2, pc2);
  pc2.data.swap(pcl_pc2.data);
}

inline void copyPointCloud2MetaData(const sensor_msgs::PointCloud2& pc2,
                                    pcl::PCLPointCloud2& pcl_pc2) {
  toPCL(pc2.header, pcl_pc2.header);
  pcl_pc2.height = pc2.height;
  pcl_pc2.width = pc2.width;
  toPCL(pc2.fields, pcl_pc2.fields);
  pcl_pc2.is_bigendian = pc2.is_bigendian;
  pcl_pc2.point_step = pc2.point_step;
  pcl_pc2.row_step = pc2.row_step;
  pcl_pc2.is_dense = pc2.is_dense;
}

inline void toPCL(const sensor_msgs::PointCloud2& pc2,
                  pcl::PCLPointCloud2& pcl_pc2) {
  copyPointCloud2MetaData(pc2, pcl_pc2);
  pcl_pc2.data = pc2.data;
}

inline void moveToPCL(sensor_msgs::PointCloud2& pc2,
                      pcl::PCLPointCloud2& pcl_pc2) {
  copyPointCloud2MetaData(pc2, pcl_pc2);
  pcl_pc2.data.swap(pc2.data);
}

} // namespace pcl_conversions

/** @} group utils */
} // namespace beam
