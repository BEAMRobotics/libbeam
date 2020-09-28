/** @file
 * @ingroup filtering
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace beam_filtering {
/**
 * @addtogroup filtering
 */

/**
 * @brief Class for the voxeldownsample filter which downsamples a point cloud
 * by converting points to a voxel grid where all points in each voxel are
 * replaced with a point in a centroid.
 */

class VoxelDownsample {
public:
  /**
   * @brief Constructor requires the parameters for the instance of the filter.
   * @param voxelSizeX Size of the voxel in the x dimension.
   * @param voxelSizeY Size of the voxel in the y dimension.
   * @param voxelSizeZ Size of the voxel in the z dimension.
   */
  VoxelDownsample(double voxelSizeX, double voxelSizeY, double voxelSizeZ);

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
  double voxelSizeX_, voxelSizeY_, vocalSizeZ_;
  /**
   * @brief Private method for breaking up clouds in the PCL PointXYZ format.
   * @param cloudIn Reference to the cloud to be broken up.
   * @return A vector of the broken up clouds.
   */
  vector<pcl::PointCloud<pcl::PointXYZ>>
      breakUpPointCloud(const pcl::PointCloud<pcl::PointXYZ>& input_cloud);

  /**
   * @brief Private method for splitting one cloud into two in the PCL PointXYZ
   * format.
   * @param cloudIn Reference to the cloud to be split.
   * @param maxAxis The axis of greatest magnitude to split the cloud.
   * @return A pair of the split clouds.
   */
  std::pair<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointXYZ>>
      splitCloudInTwo(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                      std::ptrdiff_t max_axis)
};

}