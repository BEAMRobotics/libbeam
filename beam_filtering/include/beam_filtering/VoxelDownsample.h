/** @file
 * @ingroup filtering
 */

#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudXYZPtr = PointCloudXYZ::Ptr;
using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudXYZIPtr = PointCloudXYZI::Ptr;

namespace beam_filtering {
/**
 * @addtogroup filtering
 */

/**
 * @brief VoxelDownsample is a filter for downsampling a pointcloud using a
 * voxel grid. Points in each voxel are replaced with a single point in their
 * centroid.  This is currently implemented as a wrapper over PCL's voxel grid
 * filter. If any of the voxel dimensions are set below .001m, then no filtering
 * is performed.
 *
 * PCL stores the number of voxels using 32bit integers, so integer overflow
 * protection is implemented. Clouds are split up to be filtered piecewise if
 * overflow is predicted. Midpoint splitting is used in a recursive pattern
 * until overflow is not predicted for any piece.  The output cloud is the
 * concatenation of each piece filtered.
 *
 * VoxelDownsample currently supports the PointXYZ point type with templating
 * coming soon.
 */

class VoxelDownsample {
public:
  /**
   * @brief Default constructor.
   * @param voxel_size Initial voxel size in x, y, and z.
   */
  VoxelDownsample(Eigen::Vector3f& voxel_size);

  /**
   * @brief Default destructor.
   */
  ~VoxelDownsample() = default;

  /**
   * @brief Get current voxel_size_.
   * @return Current voxel size.
   */
  Eigen::Vector3f GetVoxelSize();

  /**
   * @brief Set a new voxel_size_.
   * @param voxel_size New voxel size.
   */
  void SetVoxelSize(Eigen::Vector3f& voxel_size);

  /**
   * @brief Method for applying the filter to clouds in the PCL PointXYZ format.
   * @param input_cloud Reference to the cloud to be filtered.
   * @return The filtered cloud.
   */
  void Filter(PointCloudXYZ& input_cloud, PointCloudXYZ& output_cloud);

private:
  Eigen::Vector3f voxel_size_;

  /**
   * @brief Private method for breaking up clouds in the PCL PointXYZ format.
   * @param input_cloud Reference to the cloud to be broken up.
   * @return A vector of the broken up clouds.
   */
  std::vector<PointCloudXYZPtr>
      breakUpPointCloud(const PointCloudXYZ& input_cloud);

  /**
   * @brief Private method for splitting one cloud into two in the PCL PointXYZ
   * format.
   * @param input_cloud Reference to the cloud to be split.
   * @param max_axis The axis of greatest magnitude to split the cloud.
   * @return A pair of the split clouds.
   */
  std::pair<PointCloudXYZPtr, PointCloudXYZPtr>
      splitCloudInTwo(const PointCloudXYZ& input_cloud, int max_axis);
};

} // namespace beam_filtering