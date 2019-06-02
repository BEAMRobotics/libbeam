/** @file
 * @ingroup defects
 */

#pragma once


#include <pcl/point_types.h>

#include <beam_containers/PointBridge.h>

#include "beam_defects/Corrosion.h"
#include "beam_defects/Crack.h"
#include "beam_defects/Delam.h"
#include "beam_defects/Spall.h"

namespace beam_defects {
/** @addtogroup defects
 *  @{ */

// function to isolate crack points only
pcl::PointCloud<pcl::PointXYZ> IsolateCrackPoints(
    const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud,
    const float& threshold);

// function to isolate spall points only
pcl::PointCloud<pcl::PointXYZ> IsolateSpallPoints(
    const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud,
    const float& threshold);

// function to isolate delam points only
pcl::PointCloud<pcl::PointXYZ> IsolateDelamPoints(
    const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud,
    const float& threshold);

// function to isolate corrosion points only
pcl::PointCloud<pcl::PointXYZ> IsolateCorrosionPoints(
    const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud,
    const float& threshold);

// Extract cloud groups using euclidian segmentation
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
    GetExtractedClouds(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);

// function to extract cracks
// return type is a vector of crack objects
std::vector<beam_defects::Crack> GetCracks(
    const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud,
    const float& threshold);

// function to extract spalls
// return type is a vector of spall objects
std::vector<beam_defects::Spall> GetSpalls(
    const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud,
    const float& threshold);

// function to extract delams
// return type is a vector of delam objects
std::vector<beam_defects::Delam> GetDelams(
    const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud,
    const float& threshold);

// function to extract corrosion
// return type is a vector of delam objects
std::vector<beam_defects::Corrosion> GetCorrosion(
    const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud,
    const float& threshold);

/** @} group defects */

} // namespace beam_defects
