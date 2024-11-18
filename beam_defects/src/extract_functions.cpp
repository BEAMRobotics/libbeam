#define PCL_NO_PRECOMPILE

#include "beam_defects/extract_functions.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#include <beam_utils/kdtree.h>
#include <beam_utils/pointclouds.h>

namespace beam_defects {

// function to isolate crack points only
pcl::PointCloud<pcl::PointXYZ> IsolateCrackPoints(
    const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud,
    const float& threshold) {
  auto cloud_filtered =
      std::make_shared<pcl::PointCloud<beam_containers::PointBridge>>();
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<beam_containers::PointBridge> extract;

  for (unsigned int i = 0; i < input_cloud->points.size(); ++i) {
    if (input_cloud->points[i].crack >= threshold) {
      inliers->indices.push_back(i);
    }
  }
  extract.setInputCloud(input_cloud);
  extract.setIndices(inliers);
  extract.filter(*cloud_filtered);

  auto cloud_xyz = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  copyPointCloud(*cloud_filtered, *cloud_xyz);

  return *cloud_xyz;
};

// function to isolate spall points only
// pcl::PointCloud<pcl::PointXYZ> IsolateSpallPoints(
//     const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud,
//     const float& threshold) {
//   auto cloud_filtered =
//       std::make_shared<pcl::PointCloud<beam_containers::PointBridge>>();
//   pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
//   pcl::ExtractIndices<beam_containers::PointBridge> extract;

//   BEAM_ERROR("input_cloud.size(): {}", input_cloud->size());
//   for (unsigned int i = 0; i < input_cloud->points.size(); ++i) {
//     if (input_cloud->points[i].spall >= threshold) {
//       inliers->indices.push_back(i);
//       BEAM_ERROR("TEST");
//     }
//   }
//   extract.setInputCloud(input_cloud);
//   extract.setIndices(inliers);
//   extract.filter(*cloud_filtered);

//   auto cloud_xyz = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
//   copyPointCloud(*cloud_filtered, *cloud_xyz);

//   return *cloud_xyz;
// };

pcl::PointCloud<pcl::PointXYZ> IsolateSpallPoints(
    const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud,
    const float& threshold) {
  pcl::PointCloud<pcl::PointXYZ> cloud_isolated;
  for (auto i = 0; i < input_cloud->points.size(); ++i) {
    if (input_cloud->points[i].spall >= threshold) {
      pcl::PointXYZ p;
      p.x = input_cloud->points[i].x;
      p.y = input_cloud->points[i].y;
      p.z = input_cloud->points[i].z;
      cloud_isolated.push_back(p);
    }
  }
  return cloud_isolated;
};

// function to isolate delam points only
pcl::PointCloud<pcl::PointXYZ> IsolateDelamPoints(
    const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud,
    const float& threshold) {
  auto cloud_filtered =
      std::make_shared<pcl::PointCloud<beam_containers::PointBridge>>();
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ExtractIndices<beam_containers::PointBridge> extract;

  for (unsigned int i = 0; i < input_cloud->points.size(); ++i) {
    if (input_cloud->points[i].delam >= threshold) {
      inliers->indices.push_back(i);
    }
  }
  extract.setInputCloud(input_cloud);
  extract.setIndices(inliers);
  extract.filter(*cloud_filtered);

  auto cloud_xyz = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  copyPointCloud(*cloud_filtered, *cloud_xyz);

  return *cloud_xyz;
};

// function to isolate Corrosion points only
pcl::PointCloud<pcl::PointXYZ> IsolateCorrosionPoints(
    const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud,
    const float& threshold) {
  auto cloud_filtered =
      std::make_shared<pcl::PointCloud<beam_containers::PointBridge>>();
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ExtractIndices<beam_containers::PointBridge> extract;

  for (unsigned int i = 0; i < input_cloud->points.size(); ++i) {
    if (input_cloud->points[i].corrosion >= threshold) {
      inliers->indices.push_back(i);
    }
  }
  extract.setInputCloud(input_cloud);
  extract.setIndices(inliers);
  extract.filter(*cloud_filtered);

  auto cloud_xyz = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  copyPointCloud(*cloud_filtered, *cloud_xyz);

  return *cloud_xyz;
};

// Extract cloud groups using euclidian segmentation
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
    GetExtractedClouds(const pcl::PointCloud<pcl::PointXYZ>& input_cloud,
                       float tolerance, int min_size, int max_size) {
  std::vector<pcl::PointIndices> cluster_indices;

  // Not using beam's custom Euc Clustering, use PCL's instead
  // auto tree = std::make_shared<beam::KdTree<pcl::PointXYZ>>(input_cloud);
  // beam::ExtractEuclideanClusters<pcl::PointXYZ>(
  //     input_cloud, tree, tolerance, cluster_indices, min_size, max_size);

  auto cloud_ptr =
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(input_cloud);
  auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
  tree->setInputCloud(cloud_ptr);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(tolerance);
  ec.setMinClusterSize(min_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_ptr);
  ec.extract(cluster_indices);

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> defect_cloud;
  for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
    auto cloud_cluster = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      cloud_cluster->points.push_back(input_cloud.points[*pit]);

    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;

    defect_cloud.push_back(cloud_cluster);
  }

  return defect_cloud;
};

// function to extract cracks
// return type is a vector of crack objects
std::vector<beam_defects::Defect::Ptr> GetCracks(
    const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud,
    const float& threshold, float tolerance, int min_size, int max_size) {
  auto cloud_filtered = IsolateCrackPoints(input_cloud, threshold);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector =
      GetExtractedClouds(cloud_filtered, tolerance, min_size, max_size);

  std::vector<beam_defects::Defect::Ptr> crack_vector;
  for (auto& cloud : cloud_vector) {
    beam_defects::Defect::Ptr temp_defect =
        std::make_shared<beam_defects::Crack>();
    temp_defect->SetPointCloud(cloud);
    crack_vector.push_back(temp_defect);
  }

  return crack_vector;
};

// function to extract spalls
// return type is a vector of spall objects
std::vector<beam_defects::Defect::Ptr> GetSpalls(
    const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud,
    const float& threshold, float tolerance, int min_size, int max_size) {
  auto cloud_filtered = IsolateSpallPoints(input_cloud, threshold);
  BEAM_INFO("Extracted {} spall points from cloud of size {}",
            cloud_filtered.size(), input_cloud->size());

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector =
      GetExtractedClouds(cloud_filtered, tolerance, min_size, max_size);
  BEAM_INFO("Isolated {} spall clusters", cloud_vector.size());

  std::vector<beam_defects::Defect::Ptr> spall_vector;
  for (auto& cloud : cloud_vector) {
    beam_defects::Defect::Ptr temp_defect =
        std::make_shared<beam_defects::Spall>();
    temp_defect->SetPointCloud(cloud);
    spall_vector.push_back(temp_defect);
  }

  return spall_vector;
};

// function to extract delams
// return type is a vector of delam objects
std::vector<beam_defects::Defect::Ptr> GetDelams(
    const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud,
    const float& threshold, float tolerance, int min_size, int max_size) {
  auto cloud_filtered = IsolateDelamPoints(input_cloud, threshold);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector =
      GetExtractedClouds(cloud_filtered, tolerance, min_size, max_size);

  std::vector<beam_defects::Defect::Ptr> delam_vector;
  for (auto& cloud : cloud_vector) {
    beam_defects::Defect::Ptr temp_defect =
        std::make_shared<beam_defects::Delam>();
    temp_defect->SetPointCloud(cloud);
    delam_vector.push_back(temp_defect);
  }

  return delam_vector;
};

// function to extract corrosion
// return type is a vector of corrosion objects
std::vector<beam_defects::Defect::Ptr> GetCorrosion(
    const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud,
    const float& threshold, float tolerance, int min_size, int max_size) {
  auto cloud_filtered = IsolateCorrosionPoints(input_cloud, threshold);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector =
      GetExtractedClouds(cloud_filtered, tolerance, min_size, max_size);

  std::vector<beam_defects::Defect::Ptr> corrosion_vector;
  for (auto& cloud : cloud_vector) {
    beam_defects::Defect::Ptr temp_defect =
        std::make_shared<beam_defects::Corrosion>();
    temp_defect->SetPointCloud(cloud);
    corrosion_vector.push_back(temp_defect);
  }

  return corrosion_vector;
};

} // namespace beam_defects
