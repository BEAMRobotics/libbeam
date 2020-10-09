/** @file
 * @ingroup filtering
 */

#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace beam_filtering {
/**
 * @addtogroup filtering
 */

/**
 * @brief The VoxelDownsamplingFilter downsamples a point cloud by comparing
 * points to a voxel grid. If a voxel contains points it is replaced with a
 * point in its centroid.  This is currently implemented as a wrapper over PCL's
 * voxel grid filter. If any of the voxel dimensions are set to zero, or
 * anything below 1mm, then no filtering is performed.
 *
 * Lastly, to account for integer overflow (since pcl uses 32 bit integers for
 * building the voxel grid), this filter also checks if integer overflow would
 * occur based on the voxel size and max dimension, and iteratively breaks up
 * the pointcloud. Then the voxel grid filter is applied to all individual point
 * clouds and recombined at the end.
 */

class VoxelDownsample {
public:
  /**
   * @brief Constructor requires the parameters for the instance of the filter.
   * @param voxelSizeX Size of the voxel in the x dimension.
   * @param voxelSizeY Size of the voxel in the y dimension.
   * @param voxelSizeZ Size of the voxel in the z dimension.
   */
  VoxelDownsample(Eigen::Vector3d voxelSize);

  /**
   * @brief Default destructor.
   */
  ~VoxelDownsample() = default;

  /**
   * @brief Method for applying the filter to clouds in the PCL PointXYZ format.
   * @param input_cloud Reference to the cloud to be filtered.
   * @return The filtered cloud.
   */
  pcl::PointCloud<pcl::PointXYZ>
      Filter(pcl::PointCloud<pcl::PointXYZ>& input_cloud);

  /**
   * @brief Method for applying the filter to clouds in the PCL PointXYZI
   * format.
   * @param input_cloud Reference to the cloud to be filtered.
   * @return The filtered cloud.
   */
  pcl::PointCloud<pcl::PointXYZI>&
      Filter(pcl::PointCloud<pcl::PointXYZI>& input_cloud);

private:
  Eigen::Vector3d voxelSize_;

  /**
   * @brief Private method for breaking up clouds in the PCL PointXYZ format.
   * @param cloudIn Reference to the cloud to be broken up.
   * @return A vector of the broken up clouds.
   */
  vector<pcl::PointCloud<pcl::PointXYZ>>
      breakUpPointCloud(const pcl::PointCloud<pcl::PointXYZ>& input_cloud);

  /**
   * @brief Private method for breaking up clouds in the PCL PointXYZI format.
   * @param cloudIn Reference to the cloud to be broken up.
   * @return A vector of the broken up clouds.
   */
  vector<pcl::PointCloud<pcl::PointXYZI>>
      breakUpPointCloud(const pcl::PointCloud<pcl::PointXYZI>& input_cloud);

  /**
   * @brief Private method for splitting one cloud into two in the PCL PointXYZ
   * format.
   * @param cloudIn Reference to the cloud to be split.
   * @param maxAxis The axis of greatest magnitude to split the cloud.
   * @return A pair of the split clouds.
   */
  std::pair<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointXYZ>>
      splitCloudInTwo(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                      std::ptrdiff_t max_axis);

  /**
   * @brief Private method for splitting one cloud into two in the PCL
   * PointXYZI format.
   * @param cloudIn Reference to the cloud to be split.
   * @param maxAxis The axis of greatest magnitude to split the cloud.
   * @return A pair of the split clouds.
   */
  std::pair<pcl::PointCloud<pcl::PointXYZI>, pcl::PointCloud<pcl::PointXYZI>>
      splitCloudInTwo(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                      std::ptrdiff_t max_axis);
};

} // namespace beam_filtering