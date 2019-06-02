#include <boost/smart_ptr.hpp>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/common/common.h>

#include "beam_defects/defect_functions.h"

namespace beam_defects {

// function to calculate dot product
float dotProduct(const std::vector<float>& vect_A,
                 const std::vector<float>& vect_B) {
  return std::inner_product(vect_A.begin(), vect_A.end(), vect_B.begin(), 0.0);
}

// function to calculate cross product
std::vector<float> crossProduct(const std::vector<float>& vect_A,
                                const std::vector<float>& vect_B) {
  std::vector<float> cross_P;
  cross_P.push_back(vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1]);
  cross_P.push_back(vect_A[2] * vect_B[0] - vect_A[0] * vect_B[2]);
  cross_P.push_back(vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0]);
  return cross_P;
}

// function to calculate the length of a vector
float vectorLength(const std::vector<float>& vect_A) {
  float sum_squares =
      std::inner_product(vect_A.begin(), vect_A.end(), vect_A.begin(), 0.0);
  return std::sqrt(sum_squares);
}

// function to normalize a vector
std::vector<float> normalizeVector(const std::vector<float>& vect_A) {
  float original_length = vectorLength(vect_A);
  std::vector<float> normalized_vector;
  normalized_vector.push_back(vect_A[0] / original_length);
  normalized_vector.push_back(vect_A[1] / original_length);
  normalized_vector.push_back(vect_A[2] / original_length);
  return normalized_vector;
}

// caluclate concave hull of a point cloud
pcl::PointCloud<pcl::PointXYZ>
    calculateHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
  auto cloud_hull = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
  concave_hull.setInputCloud(input_cloud);
  concave_hull.setAlpha(0.1); // limits the size of the hull segments. Smaller
                              // alpha means more detailed hull
  concave_hull.reconstruct(*cloud_hull);

  return *cloud_hull;
}

// Calculate Normal Vector of a plane
std::vector<float>
    planeNormalVector(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> sac_segmentor;
  // Optional
  sac_segmentor.setOptimizeCoefficients(true);
  // Mandatory
  sac_segmentor.setModelType(pcl::SACMODEL_PLANE);
  sac_segmentor.setMethodType(pcl::SAC_RANSAC);
  sac_segmentor.setDistanceThreshold(0.01);

  sac_segmentor.setInputCloud(input_cloud);
  sac_segmentor.segment(inliers, coefficients);

  // Extract the plane normal vector from the coefficients
  std::vector<float> plane_norm_vect;
  plane_norm_vect.push_back(coefficients.values[0]);
  plane_norm_vect.push_back(coefficients.values[1]);
  plane_norm_vect.push_back(coefficients.values[2]);

  return plane_norm_vect;
}

// Project points from a cloud onto a common plane
pcl::PointCloud<pcl::PointXYZ>
    project2Plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                  const std::vector<float>& plane_norm_vect) {
  int num_dims = plane_norm_vect.size();

  // initialize normal vectors to represent new axes
  std::vector<float> norm_new_x;
  std::vector<float> norm_new_y;
  std::vector<float> norm_new_z;

  // initialize orignal x_axis for cross product projection
  std::vector<float> x_axis(num_dims, 0);
  x_axis[0] = 1;

  if (plane_norm_vect == x_axis) {
    // initialize original y_axis for cross product projection
    std::vector<float> y_axis(num_dims,0);
    y_axis[1] = 1;

    //Generate new x axis by crossing the old y with the plane normal
    // this assumes that the plane normal is the new z axis
    std::vector<float> new_x = crossProduct(y_axis, plane_norm_vect);

    // Generate new y axis by prossing the plane normal (new z) with the new x
    std::vector<float> new_y = crossProduct(plane_norm_vect, new_x);

    // Make all vectors into unit vectors
    norm_new_x = normalizeVector(new_x);
    norm_new_y = normalizeVector(new_y);
    norm_new_z = normalizeVector(plane_norm_vect);

  } else {
    // Generate new y axis by crossing the old x with the plane normal
    // this assumes that the plane normal is the new z axis
    std::vector<float> new_y = crossProduct(x_axis, plane_norm_vect);

    // Generate new x axis by crossing the plane normal (new z) with the new y
    std::vector<float> new_x = crossProduct(plane_norm_vect, new_y);

    // Make all new axis vectors into unit vectors
    norm_new_x = normalizeVector(new_x);
    norm_new_y = normalizeVector(new_y);
    norm_new_z = normalizeVector(plane_norm_vect);
  }

  // Project each point to the new coordinate space using dot product
  // This assumes that the new origin is still (0,0,0)
  for (auto& point : input_cloud->points) {
    std::vector<float> tmp_point = {point.x, point.y, point.z};

    point.x = dotProduct(tmp_point, norm_new_x);
    point.y = dotProduct(tmp_point, norm_new_y);
    point.z = dotProduct(tmp_point, norm_new_z);
  }

  return *input_cloud;
}

// calculate area given a hull cloud
float calculateHullArea(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
  double area = 0.0;
  for (unsigned int i = 0; i < input_cloud->points.size(); ++i) {
    int j = (i + 1) % input_cloud->points.size();
    area += 0.5 * (input_cloud->points[i].x * input_cloud->points[j].y -
                   input_cloud->points[j].x * input_cloud->points[i].y);
  }

  area = std::abs(area);

  return area;
}

// calculate maximum length from a hull cloud
float calculateMaxLength(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D (*input_cloud, minPt, maxPt);
  double dx = maxPt.x - minPt.x;
  double dy = maxPt.y - minPt.y;
  double length = std::sqrt(pow(dx,2) + pow(dy,2));

  return length;
}

} // namespace beam_defects
