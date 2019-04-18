/** @file
 * @ingroup defects
 */

#pragma once

#include <boost/smart_ptr.hpp>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <cmath>
#include <vector>

#include <beam_containers/PointBridge.h>

#include "beam_defects/Crack.h"
#include "beam_defects/Delam.h"
#include "beam_defects/Spall.h"

namespace beam_defects {
/** @addtogroup defects
  *  @{ */

// function to extract cracks
// return type is a vector of crack objects
std::vector<beam_defects::Crack> GetCracks(const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud);

// function to extract spalls
// return type is a vector of spall objects
std::vector<beam_defects::Spall> GetSpalls(const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud);

// function to extract delams
// return type is a vector of delam objects
std::vector<beam_defects::Delam> GetDelams(const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud);

// function to isolate crack points only
pcl::PointCloud<beam_containers::PointBridge> IsolateCrackPoints(const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud);

// function to isolate spall points only
pcl::PointCloud<beam_containers::PointBridge> IsolateSpallPoints(const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud);

// function to isolate delam points only
pcl::PointCloud<beam_containers::PointBridge> IsolateDelamPoints(const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud);

// Extract cloud groups using euclidian segmentation
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> GetExtractedClouds(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);

/** @} group defects */

} // namespace beam_defects