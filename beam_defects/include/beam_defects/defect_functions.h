/** @file
 * @ingroup defects
 */

#pragma once

#include <boost/smart_ptr.hpp>

#include <pcl/common/common.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

#include <cmath>
#include <typeinfo>
#include <vector>

namespace beam_defects {
/** @addtogroup defects
 *  @{ */

/**
 * @brief Function to calculate the dot product of two vectors
 * @param vect_A
 * @param vect_B
 * @return Returns scalar dot product
 */
float DotProduct(const std::vector<float>& vect_A,
                 const std::vector<float>& vect_B);

/**
 * @brief Function to caclulate the cross product of two vectors
 * @param vect_A
 * @param vect_B
 * @return Returns vector result of cross product
 */
std::vector<float> CrossProduct(const std::vector<float>& vect_A,
                                const std::vector<float>& vect_B);

/**
 * @brief Function to calculate the length of a vector
 * @param vect_A
 * @return Returns scalar vector length
 */
float VectorLength(const std::vector<float>& vect_A);

/**
 * @brief Function to normalize a vector
 * @param vect_A
 * @return Returns normalized unit vector
 */
std::vector<float> NormalizeVector(const std::vector<float>& vect_A);

/**
 * @brief Function to remove noise from a point cloud using RANSAC
 * @details RANSAC used to fit plane to input_cloud. Points not within X meters
 * of fitted plane are removed
 * @param input_cloud
 * @param outlier_threshold The threshold (in meters) for outlier removal
 * @return Returns a pcl XYZ point cloud with outlier points removed
 */
pcl::PointCloud<pcl::PointXYZ>
    PCNoiseRemoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                   float outlier_threshold = 0.01);

/**
 * @brief Function to extract the concave hull of an input point cloud
 * @param input_cloud
 * @param alpha Variable to limit the size of the resultant hull segments (the
 * smaller, the more detailed the hull)
 * @return Returns the concave hull of the input point cloud
 */
pcl::PointCloud<pcl::PointXYZ>
    ConcaveHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                float alpha = 0.1);

/**
 * @brief Function to extract the convex hull of an input point cloud
 * @param input_cloud
 * @return Returns the convex hull of the input point cloud
 */
pcl::PointCloud<pcl::PointXYZ>
    ConvexHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);

/**
 * @brief Function to calculate the normal vector to a point cloud plane
 * @param input_cloud
 * @return Returns the normal vector of the plane fit to the point cloud
 */
std::vector<float>
    PlaneNormalVector(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);

/**
 * @brief Function to rotate a 3D point cloud plane to a 2D axis
 * @param input_cloud
 * @param plane_norm_vect
 * @return Returns a point cloud rotated to the x,y plane
 */
pcl::PointCloud<pcl::PointXYZ>
    Project2Plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                  const std::vector<float>& plane_norm_vect);

/**
 * @brief Function to calculate the area bounded by a 2D point cloud hull
 * @param input_cloud
 * @return Returns the scalar area bounded by the input hull
 */
float HullArea(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);

/**
 * @brief Function to calculate the diagonal length between the maximum x,y and
 * minimum x,y of the point cloud bounding box
 * @param input_cloud
 * @return Returns the scalar maximum length
 */
float MaxLength(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);

/** @} group defects */

} // namespace beam_defects
