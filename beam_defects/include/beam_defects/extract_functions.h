/** @file
 * @ingroup defects
 */

#pragma once

#include "beam_defects/Corrosion.h"
#include "beam_defects/Crack.h"
#include "beam_defects/Delam.h"
#include "beam_defects/Spall.h"

#include <beam_containers/PointBridge.h>

#include <pcl/point_types.h>

#include <vector>

namespace beam_defects {
/** @addtogroup defects
 *  @{ */

/**
 * @brief Function to isolate points above a certain crack value threshold
 * @param input_cloud
 * @param threshold Value between 0 and 1. All points with crack value >
 * threshold are kept in the point cloud
 * @return Returns a point cloud with only the points above a certain crack
 * value threshold
 */
pcl::PointCloud<pcl::PointXYZ> IsolateCrackPoints(
    const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud,
    float threshold);

/**
 * @brief Function to isolate points above a certain spall value threshold
 * @param input_cloud
 * @param threshold Value between 0 and 1. All points with spall value >
 * threshold are kept in the point cloud
 * @return Returns a point cloud with only the points above a certain spall
 * value threshold
 */
pcl::PointCloud<pcl::PointXYZ> IsolateSpallPoints(
    const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud,
    float threshold);

/**
 * @brief Function to isolate points above a certain delam value threshold
 * @param input_cloud
 * @param threshold Value between 0 and 1. All points with delam value >
 * threshold are kept in the point cloud
 * @return Returns a point cloud with only the points above a certain delam
 * value threshold
 */
pcl::PointCloud<pcl::PointXYZ> IsolateDelamPoints(
    const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud,
    float threshold);

/**
 * @brief Function to isolate points above a certain corrosion value threshold
 * @param input_cloud
 * @param threshold Value between 0 and 1. All points with corrosion value >
 * threshold are kept in the point cloud
 * @return Returns a point cloud with only the points above a certain corrosion
 * value threshold
 */
pcl::PointCloud<pcl::PointXYZ> IsolateCorrosionPoints(
    const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud,
    float threshold);

/**
 * @brief Function using euclidean distance to segment individual defect clouds
 * @param input_cloud
 * @param tolerance L2 spatial tolerance for segmenting cluster. Default =
 * 0.075m
 * @param min_size Minimum number of points to be considered a cluster. Default
 * = 100 points
 * @return Returns a vector of defect point clouds
 */
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
    GetExtractedClouds(const pcl::PointCloud<pcl::PointXYZ>& input_cloud,
                       float tolerance = 0.075, int min_size = 100);

/**
 * @brief Function to extract crack objects from an input cloud
 * @param input_cloud
 * @param threshold Value between 0 and 1. All points with crack value >
 * threshold are considered crack points in the extraction algorithm
 * @param tolerance L2 spatial tolerance for segmenting cluster. Default =
 * 0.075m
 * @param min_size Minimum number of points to be considered a cluster. Default
 * = 100 points
 * @return Returns a vector of crack objects
 */
std::vector<beam_defects::Defect::Ptr> GetCracks(
    const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud,
    float threshold, float tolerance = 0.075, int min_size = 100,
    float hull_alpha = 0.1);

/**
 * @brief Function to extract spall objects from an input cloud
 * @param input_cloud
 * @param threshold Value between 0 and 1. All points with spall value >
 * threshold are considered spall points in the extraction algorithm
 * @param tolerance L2 spatial tolerance for segmenting cluster. Default =
 * 0.075m
 * @param min_size Minimum number of points to be considered a cluster. Default
 * = 100 points
 * @return Returns a vector of spall objects
 */
std::vector<beam_defects::Defect::Ptr> GetSpalls(
    const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud,
    float threshold, float tolerance = 0.075, int min_size = 100,
    float hull_alpha = 0.1);

/**
 * @brief Function to extract delam objects from an input cloud
 * @param input_cloud
 * @param threshold Value between 0 and 1. All points with delam value >
 * threshold are considered delam points in the extraction algorithm
 * @param tolerance L2 spatial tolerance for segmenting cluster. Default =
 * 0.075m
 * @param min_size Minimum number of points to be considered a cluster. Default
 * = 100 points
 * @return Returns a vector of delam objects
 */
std::vector<beam_defects::Defect::Ptr> GetDelams(
    const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud,
    float threshold, float tolerance = 0.075, int min_size = 100,
    float hull_alpha = 0.1);

/**
 * @brief Function to extract corrosion objects from an input cloud
 * @param input_cloud
 * @param threshold Value between 0 and 1. All points with corrosion value >
 * threshold are considered corrosion points in the extraction algorithm
 * @param tolerance L2 spatial tolerance for segmenting cluster. Default =
 * 0.075m
 * @param min_size Minimum number of points to be considered a cluster. Default
 * = 100 points
 * @return Returns a vector of corrosion objects
 */
std::vector<beam_defects::Defect::Ptr> GetCorrosion(
    const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud,
    float threshold, float tolerance = 0.075, int min_size = 100,
    float hull_alpha = 0.1);

/** @} group defects */

} // namespace beam_defects
