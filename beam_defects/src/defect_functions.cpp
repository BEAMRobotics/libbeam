#include "beam_defects/defect_functions.h"
#include <beam_utils/pointclouds.h>

#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/search/kdtree.h>

#include <boost/smart_ptr.hpp>


namespace beam_defects {

// function to calculate dot product
float DotProduct(const std::vector<float>& vect_A,
                 const std::vector<float>& vect_B) {
  return std::inner_product(vect_A.begin(), vect_A.end(), vect_B.begin(), 0.0);
}

// function to calculate cross product
std::vector<float> CrossProduct(const std::vector<float>& vect_A,
                                const std::vector<float>& vect_B) {
  std::vector<float> cross_P;
  cross_P.push_back(vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1]);
  cross_P.push_back(vect_A[2] * vect_B[0] - vect_A[0] * vect_B[2]);
  cross_P.push_back(vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0]);
  return cross_P;
}

// function to calculate the length of a vector
float VectorLength(const std::vector<float>& vect_A) {
  float sum_squares =
      std::inner_product(vect_A.begin(), vect_A.end(), vect_A.begin(), 0.0);
  return std::sqrt(sum_squares);
}

// function to normalize a vector
std::vector<float> NormalizeVector(const std::vector<float>& vect_A) {
  float original_length = VectorLength(vect_A);
  std::vector<float> normalized_vector;
  normalized_vector.push_back(vect_A[0] / original_length);
  normalized_vector.push_back(vect_A[1] / original_length);
  normalized_vector.push_back(vect_A[2] / original_length);
  return normalized_vector;
}

// RANSAC noise removal
pcl::PointCloud<pcl::PointXYZ>
    PCNoiseRemoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                   float outlier_threshold) {
  auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto coefficients = std::make_shared<pcl::ModelCoefficients>();
  auto inliers = std::make_shared<pcl::PointIndices>();
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(outlier_threshold);

  seg.setInputCloud(input_cloud);
  seg.segment(*inliers, *coefficients);

  // Project the model inliers
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setIndices(inliers);
  proj.setInputCloud(input_cloud);
  proj.setModelCoefficients(coefficients);
  proj.filter(*cloud_filtered);

  return *cloud_filtered;
}

// caluclate concave hull of a point cloud
pcl::PointCloud<pcl::PointXYZ>
    ConcaveHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                float alpha) {
  auto cloud_hull = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
  concave_hull.setInputCloud(input_cloud);
  concave_hull.setAlpha(alpha); // limits the size of the hull segments. Smaller
                                // alpha means more detailed hull
  concave_hull.reconstruct(*cloud_hull);

  return *cloud_hull;
}

// caluclate convex hull of a point cloud
pcl::PointCloud<pcl::PointXYZ>
    ConvexHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
  auto cloud_hull = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::ConvexHull<pcl::PointXYZ> convex_hull;
  convex_hull.setInputCloud(input_cloud);
  convex_hull.reconstruct(*cloud_hull);

  return *cloud_hull;
}

// Calculate Normal Vector of a plane
std::vector<float>
    PlaneNormalVector(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
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
    Project2Plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
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
    std::vector<float> y_axis(num_dims, 0);
    y_axis[1] = 1;

    // Generate new x axis by crossing the old y with the plane normal
    // this assumes that the plane normal is the new z axis
    std::vector<float> new_x = CrossProduct(y_axis, plane_norm_vect);

    // Generate new y axis by prossing the plane normal (new z) with the new x
    std::vector<float> new_y = CrossProduct(plane_norm_vect, new_x);

    // Make all vectors into unit vectors
    norm_new_x = NormalizeVector(new_x);
    norm_new_y = NormalizeVector(new_y);
    norm_new_z = NormalizeVector(plane_norm_vect);

  } else {
    // Generate new y axis by crossing the old x with the plane normal
    // this assumes that the plane normal is the new z axis
    std::vector<float> new_y = CrossProduct(x_axis, plane_norm_vect);

    // Generate new x axis by crossing the plane normal (new z) with the new y
    std::vector<float> new_x = CrossProduct(plane_norm_vect, new_y);

    // Make all new axis vectors into unit vectors
    norm_new_x = NormalizeVector(new_x);
    norm_new_y = NormalizeVector(new_y);
    norm_new_z = NormalizeVector(plane_norm_vect);
  }

  // Project each point to the new coordinate space using dot product
  // This assumes that the new origin is still (0,0,0)
  for (auto& point : input_cloud->points) {
    std::vector<float> tmp_point = {point.x, point.y, point.z};

    point.x = DotProduct(tmp_point, norm_new_x);
    point.y = DotProduct(tmp_point, norm_new_y);
    point.z = DotProduct(tmp_point, norm_new_z);
  }

  return *input_cloud;
}

// calculate area given a hull cloud
float HullArea(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
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
float MaxLength(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
  pcl::PointXYZ minPt, maxPt;
  beam::getMinMax3D(*input_cloud, minPt, maxPt);
  double dx = maxPt.x - minPt.x;
  double dy = maxPt.y - minPt.y;
  double length = std::sqrt(pow(dx, 2) + pow(dy, 2));

  return length;
}

float HausdorffDist(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_a,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_b) {
  float hausdorff_dist = 0;
  pcl::search::KdTree<pcl::PointXYZ> tree;
  tree.setInputCloud(cloud_b);
  float max_dist = -std::numeric_limits<float>::max();
  for (const auto &point : cloud_a->points) {
    std::vector<int> indices (1);
    std::vector<float> sqr_distances (1);

    tree.nearestKSearch(point, 1, indices, sqr_distances);
    if (sqr_distances[0] > max_dist) {
      max_dist = sqr_distances[0];
    }
  }
  
  hausdorff_dist = std::sqrt(max_dist);

  return hausdorff_dist;
}

} // namespace beam_defects
