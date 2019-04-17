#include "beam_defects/extract_functions.h"

namespace beam_defects {
// function to extract cracks
// return type is a vector of crack objects
std::vector<beam_defects::Crack> GetCracks(const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud){
  auto cloud_filtered = boost::make_shared<pcl::PointCloud<beam_containers::PointBridge>>();
  *cloud_filtered = IsolateCrackPoints(input_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  copyPointCloud(*cloud_filtered, *cloud_xyz);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector = GetExtractedClouds(cloud_xyz);

  std::vector<beam_defects::Crack> crack_vector;
  for (auto& cloud : cloud_vector) {
    beam_defects::Crack temp = beam_defects::Crack(cloud);
    crack_vector.push_back(temp);
  }

  return crack_vector;
};

// function to extract spalls
// return type is a vector of spall objects
std::vector<beam_defects::Spall> GetSpalls(const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud){
  std::vector<beam_defects::Spall> spall_vector;
  return spall_vector;
};

// function to extract delams
// return type is a vector of delam objects
std::vector<beam_defects::Delam> GetDelams(const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud){
  std::vector<beam_defects::Delam> delam_vector;
  return delam_vector;
};

// function to isolate crack points only
pcl::PointCloud<beam_containers::PointBridge> IsolateCrackPoints(const pcl::PointCloud<beam_containers::PointBridge>::Ptr& input_cloud){
  auto cloud_filtered = boost::make_shared<pcl::PointCloud<beam_containers::PointBridge>>();
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<beam_containers::PointBridge> extract;

  for (unsigned int i = 0; i < input_cloud->points.size (); ++i){
    if(input_cloud->points[i].crack >= 0.9){
        inliers->indices.push_back(i);
      }
  }
  extract.setInputCloud(input_cloud);
  extract.setIndices(inliers);
  extract.filter(*cloud_filtered);

  return *cloud_filtered;
};

//typedef <pcl::PointCloud<pcl::PointXYZ> PointCloud;
//typedef std::vector<PointCloud> PointCloudVec;

// Extract cloud groups using euclidian segmentation
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> GetExtractedClouds(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud){
  auto tree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
  tree->setInputCloud (input_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.05); // 5cm
  ec.setMinClusterSize (50);
  ec.setMaxClusterSize (50000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (input_cloud);
  ec.extract (cluster_indices);

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> defect_cloud;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    // auto cloud_cluster = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      cloud_cluster->points.push_back (input_cloud->points[*pit]);
    }

    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    defect_cloud.push_back(cloud_cluster);
  }

  return defect_cloud;
};

} // namespace beam_defects
