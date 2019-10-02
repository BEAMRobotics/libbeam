/** @file
 * @ingroup matching
 */

#ifndef BEAM_PCL_COMMON_HPP
#define BEAM_PCL_COMMON_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

namespace beam_matching {
/** @addtogroup matching
 *  @{ */

/* This type definition is used as a shorthand for the pointcloud object type
 * used
 * by the scan matching implementations in PCL. A number of those
 * implementations are
 * wrapped, so they use the same datatype for the pointcloud.
 */
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCLPointCloudPtr;

/** @} group matching */
}  // namespace beam_matching

#endif  // BEAM_MATCHING_PCL_COMMON_HPP
