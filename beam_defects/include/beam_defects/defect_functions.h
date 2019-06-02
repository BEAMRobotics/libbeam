/** @file
 * @ingroup defects
 */

#pragma once

#include <pcl/point_types.h>

namespace beam_defects {
/** @addtogroup defects
 *  @{ */

// function to calculate dot product
float dotProduct(const std::vector<float>& vect_A,
                 const std::vector<float>& vect_B);

// function to calculate cross product
std::vector<float> crossProduct(const std::vector<float>& vect_A,
                                const std::vector<float>& vect_B);

// function to calculate the length of a vector
float vectorLength(const std::vector<float>& vect_A);

// function to normalize a vector
std::vector<float> normalizeVector(const std::vector<float>& vect_A);

// caluclate concave hull of a point cloud
pcl::PointCloud<pcl::PointXYZ>
    calculateHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);

// Calculate Normal Vector of a plane
std::vector<float>
    planeNormalVector(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);

// Project points from a cloud onto a common plane
pcl::PointCloud<pcl::PointXYZ>
    project2Plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                  const std::vector<float>& plane_norm_vect);

// calculate area given a hull cloud
float calculateHullArea(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);

// calculate maximum length from a hull cloud
float calculateMaxLength(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);

/** @} group defects */

} // namespace beam_defects
