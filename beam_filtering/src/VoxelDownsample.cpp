#include "beam_filtering/VoxelDownsample.h"

#include <boost/smart_ptr.hpp>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

namespace beam_filtering {

VoxelDownsample::VoxelDownsample(const Eigen::Vector3f& voxel_size) {
  voxel_size_ = voxel_size;
}

Eigen::Vector3f VoxelDownsample::GetVoxelSize() const {
  return voxel_size_;
}

void VoxelDownsample::SetVoxelSize(const Eigen::Vector3f& voxel_size) {
  voxel_size_ = voxel_size;
}

void VoxelDownsample::Filter(const PointCloudXYZ& input_cloud,
                             PointCloudXYZ& output_cloud) {
  output_cloud.clear();

  // Filter cloud in pieces to prevent integer overflow.
  std::vector<PointCloudXYZPtr> broken_cloud_ptrs =
      BreakUpPointCloud(input_cloud);
  // Create a pcl voxel grid filter and use voxel_size_ as grid size.
  pcl::VoxelGrid<pcl::PointXYZ> downsampler;
  downsampler.setLeafSize(voxel_size_[0], voxel_size_[1], voxel_size_[2]);
  for (PointCloudXYZPtr cloud_ptr : broken_cloud_ptrs) {
    PointCloudXYZPtr filtered_cloud_ptr = boost::make_shared<PointCloudXYZ>();
    downsampler.setInputCloud(cloud_ptr);
    downsampler.filter(*filtered_cloud_ptr);
    output_cloud += *filtered_cloud_ptr;
  }
}

std::vector<PointCloudXYZPtr>
    VoxelDownsample::BreakUpPointCloud(const PointCloudXYZ& input_cloud) {
  // Determine if integer overflow will occur.
  pcl::PointXYZ min;
  pcl::PointXYZ max;
  pcl::getMinMax3D(input_cloud, min, max);
  Eigen::Vector3f axis_dimensions(std::abs(max.x - min.x),
                                  std::abs(max.y - min.y),
                                  std::abs(max.z - min.z));
  uint64_t voxel_count_x =
      static_cast<uint64_t>((axis_dimensions[0] / voxel_size_[0]) + 1);
  uint64_t voxel_count_y =
      static_cast<uint64_t>((axis_dimensions[1] / voxel_size_[1]) + 1);
  uint64_t voxel_count_z =
      static_cast<uint64_t>((axis_dimensions[2] / voxel_size_[2]) + 1);
  if ((voxel_count_x * voxel_count_y * voxel_count_z) >
      static_cast<uint64_t>(std::numeric_limits<std::int32_t>::max())) {
    // Split cloud along max axis until integer overflow does not occur.
    int max_axis;
    axis_dimensions.maxCoeff(&max_axis);
    std::pair<PointCloudXYZPtr, PointCloudXYZPtr> split_clouds_ptrs =
        SplitCloudInTwo(input_cloud, max_axis);
    // Recursively call BreakUpPointCloud on each split.
    std::vector<PointCloudXYZPtr> cloud_ptrs_1 =
        BreakUpPointCloud(*(split_clouds_ptrs.first));
    std::vector<PointCloudXYZPtr> cloud_ptrs_2 =
        BreakUpPointCloud(*(split_clouds_ptrs.second));
    cloud_ptrs_1.insert(cloud_ptrs_1.end(), cloud_ptrs_2.begin(),
                        cloud_ptrs_2.end());
    return cloud_ptrs_1;
  } else {
    // No overflow, return output cloud pointers in vector.
    std::vector<PointCloudXYZPtr> output_cloud_ptrs;
    PointCloudXYZPtr output_cloud_ptr =
        boost::make_shared<PointCloudXYZ>(input_cloud);
    output_cloud_ptrs.push_back(output_cloud_ptr);
    return output_cloud_ptrs;
  }
}

std::pair<PointCloudXYZPtr, PointCloudXYZPtr>
    VoxelDownsample::SplitCloudInTwo(const PointCloudXYZ& input_cloud,
                                     const int max_axis) {
  // Get mid point of cloud.
  pcl::PointXYZ min;
  pcl::PointXYZ max;
  pcl::getMinMax3D(input_cloud, min, max);
  Eigen::Vector3f eigen_min(min.x, min.y, min.z);
  Eigen::Vector3f eigen_max(max.x, max.y, max.z);
  float midpoint = (eigen_max[max_axis] + eigen_min[max_axis]) / 2;

  // Create clouds for points (1) < than midpoint (2) > than midpoint.
  PointCloudXYZPtr cloud_1_ptr = boost::make_shared<PointCloudXYZ>();
  PointCloudXYZPtr cloud_2_ptr = boost::make_shared<PointCloudXYZ>();
  for (pcl::PointXYZ point : input_cloud) {
    Eigen::Vector3f eigen_point(point.x, point.y, point.z);
    if (eigen_point[max_axis] < midpoint) {
      cloud_1_ptr->push_back(point);
    } else {
      cloud_2_ptr->push_back(point);
    }
  }
  return std::pair<PointCloudXYZPtr, PointCloudXYZPtr>(cloud_1_ptr,
                                                       cloud_2_ptr);
}

} // namespace beam_filtering
