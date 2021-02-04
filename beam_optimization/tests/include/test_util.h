#pragma once

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "beam_calibration/CameraModel.h"

namespace beam_optimization {

typedef Eigen::aligned_allocator<Eigen::Vector4d> AlignVec4d;
typedef Eigen::aligned_allocator<Eigen::Vector3d> AlignVec3d;
typedef Eigen::aligned_allocator<Eigen::Vector2d> AlignVec2d;
typedef Eigen::aligned_allocator<Eigen::Affine3d> AlignAff3d;
typedef Eigen::aligned_allocator<Eigen::Matrix4d> AlignMat4d;

namespace test_util {

void OutputTransformInformation(const Eigen::Affine3d& T,
                                const std::string& transform_name);

void OutputTransformInformation(const Eigen::Matrix4d& T,
                                const std::string& transform_name);

pcl::PointCloud<pcl::PointXYZ>::Ptr MakePointCloud(const std::vector<Eigen::Vector4d, AlignVec4d> &points);
pcl::PointCloud<pcl::PointXYZ>::Ptr MakePointCloud(const std::vector<Eigen::Vector4d> &points);

pcl::PointCloud<pcl::PointXYZ>::Ptr MakePointCloud(const std::vector<Eigen::Vector2d, AlignVec2d> &points);
pcl::PointCloud<pcl::PointXYZ>::Ptr MakePointCloud(const std::vector<Eigen::Vector2d> &points);


inline Eigen::Vector3d PCLPointToEigen(const pcl::PointXYZ& pt_in) {
  return Eigen::Vector3d(pt_in.x, pt_in.y, pt_in.z);
}

inline Eigen::Vector2d PCLPixelToEigen(const pcl::PointXY& pt_in) {
  return Eigen::Vector2d(pt_in.x, pt_in.y);
}

inline pcl::PointXYZ EigenPointToPCL(const Eigen::Vector3d& pt_in) {
  pcl::PointXYZ pt_out;
  pt_out.x = pt_in[0];
  pt_out.y = pt_in[1];
  pt_out.z = pt_in[2];
  return pt_out;
}

inline pcl::PointXY EigenPixelToPCL(const Eigen::Vector2d& pt_in) {
  pcl::PointXY pt_out;
  pt_out.x = pt_in[0];
  pt_out.y = pt_in[1];
  return pt_out;
}

} // namespace util

} // namespace beam_optimization