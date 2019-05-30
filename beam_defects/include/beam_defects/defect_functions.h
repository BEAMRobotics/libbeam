/** @file
 * @ingroup defects
 */

#pragma once

#include <boost/smart_ptr.hpp>

#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/point_types.h>
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

// function to calculate dot product
float DotProduct(const std::vector<float>& vect_A,
                 const std::vector<float>& vect_B);

// function to calculate cross product
std::vector<float> CrossProduct(const std::vector<float>& vect_A,
                                const std::vector<float>& vect_B);

// function to calculate the length of a vector
float VectorLength(const std::vector<float>& vect_A);

// function to normalize a vector
std::vector<float> NormalizeVector(const std::vector<float>& vect_A);
// RANSAC noise removal
pcl::PointCloud<pcl::PointXYZ>
    PCNoiseRemoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);

// caluclate concave hull of a point cloud
pcl::PointCloud<pcl::PointXYZ>
    ConcaveHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);

// caluclate convex hull of a point cloud
pcl::PointCloud<pcl::PointXYZ>
    ConvexHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);

// Calculate Normal Vector of a plane
std::vector<float>
    PlaneNormalVector(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);

// Project points from a cloud onto a common plane
pcl::PointCloud<pcl::PointXYZ>
    Project2Plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                  const std::vector<float>& plane_norm_vect);

// calculate area given a hull cloud
float HullArea(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);

// calculate maximum length from a hull cloud
float MaxLength(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);

/** @} group defects */

} // namespace beam_defects
