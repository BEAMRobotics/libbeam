/** @file
 * @ingroup filtering
 */

// PCL specific headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#pragma once

namespace beam_filtering {
/** @addtogroup filtering
 *  @{ */

/**
 * @brief class for the voxeldownsample filter which downsamples a point cloud
 * by converting points to a voxel grid where all points in each voxel are
 * replaced with a point in a centroid.
 */

class VoxelDownsample {
public:
  /**
   * @brief constructor
   */
  VoxelDownsample();

  /**
   * @brief Default destructor
   */
  ~VoxelDownsample() = default;

  /**
   * @brief Method for applying the filter for PointXYZ format
   * @param input_cloud cloud to be filtered
   * @param filtered_cloud cloud to save to
   */
  void Filter(pcl::PointCloud<pcl::PointXYZ>& input_cloud,
              pcl::PointCloud<pcl::PointXYZ>& filtered_cloud);

  /**
   * @brief Method for applying the filter for PointXYZI format
   * @param input_cloud cloud to be filtered
   * @param filtered_cloud cloud to save to
   */
  void Filter(pcl::PointCloud<pcl::PointXYZI>& input_cloud,
              pcl::PointCloud<pcl::PointXYZI>& filtered_cloud);

private:
  vector<PointCloudPtr> breakUpPointCloud(const PointCloud& cloudIn);
  std::pair<PointCloudPtr, PointCloudPtr>
      splitCloudInTwo(const PointCloud& cloud, std::ptrdiff_t maxAxis)
};

/** @} group filtering */

} // namespace beam_filtering