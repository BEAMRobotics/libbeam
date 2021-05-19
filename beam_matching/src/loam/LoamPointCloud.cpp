#include <beam_matching/loam/LoamPointCloud.h>

#include <Eigen/Geometry>
#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <beam_utils/log.h>

namespace beam_matching {

void LoamFeatures::Clear() {
  strong.Clear();
  weak.Clear();
}

void LoamFeatureCloud::Clear() {
  cloud.clear();
  ClearKDTree();
}

void LoamFeatureCloud::ClearKDTree() {
  kdtree = pcl::KdTreeFLANN<pcl::PointXYZ>();
  kdtree_empty = true;
}

void LoamFeatureCloud::BuildKDTree(bool override_tree) {
  // if tree is not empty, and we do not want to override, then do nothing
  if (!kdtree_empty && !override_tree) { return; }

  kdtree = pcl::KdTreeFLANN<pcl::PointXYZ>();
  kdtree.setInputCloud(std::make_shared<PointCloud>(cloud));
  kdtree_empty = false;
}

LoamPointCloud::LoamPointCloud(const PointCloud& edge_features_strong,
                               const PointCloud& surface_features_strong,
                               const PointCloud& edge_features_weak,
                               const PointCloud& surface_features_weak) {
  edges.strong.cloud = edge_features_strong;
  surfaces.strong.cloud = surface_features_strong;
  edges.weak.cloud = edge_features_weak;
  surfaces.weak.cloud = surface_features_weak;
}

void LoamPointCloud::AddSurfaceFeaturesStrong(const PointCloud& new_features,
                                              const Eigen::Matrix4d& T) {
  if (!T.isIdentity()) {
    PointCloud new_features_transformed;
    pcl::transformPointCloud(new_features, new_features_transformed, T);
    surfaces.strong.cloud += new_features_transformed;
    return;
  }
  surfaces.strong.cloud += new_features;
  return;
}

void LoamPointCloud::AddEdgeFeaturesStrong(const PointCloud& new_features,
                                           const Eigen::Matrix4d& T) {
  if (!T.isIdentity()) {
    PointCloud new_features_transformed;
    pcl::transformPointCloud(new_features, new_features_transformed, T);
    edges.strong.cloud += new_features_transformed;
    return;
  }
  edges.strong.cloud += new_features;
  return;
}

void LoamPointCloud::AddSurfaceFeaturesWeak(const PointCloud& new_features,
                                            const Eigen::Matrix4d& T) {
  if (!T.isIdentity()) {
    PointCloud new_features_transformed;
    pcl::transformPointCloud(new_features, new_features_transformed, T);
    surfaces.weak.cloud += new_features_transformed;
    return;
  }
  surfaces.weak.cloud += new_features;
  return;
}

void LoamPointCloud::AddEdgeFeaturesWeak(const PointCloud& new_features,
                                         const Eigen::Matrix4d& T) {
  if (!T.isIdentity()) {
    PointCloud new_features_transformed;
    pcl::transformPointCloud(new_features, new_features_transformed, T);
    edges.weak.cloud += new_features_transformed;
    return;
  }
  edges.weak.cloud += new_features;
  return;
}

void LoamPointCloud::TransformPointCloud(const Eigen::Matrix4d& T) {
  if (edges.strong.cloud.size() > 0) {
    pcl::transformPointCloud(edges.strong.cloud, edges.strong.cloud, T);
    edges.strong.ClearKDTree();
  }
  if (edges.weak.cloud.size() > 0) {
    pcl::transformPointCloud(edges.weak.cloud, edges.weak.cloud, T);
    edges.weak.ClearKDTree();
  }
  if (surfaces.strong.cloud.size() > 0) {
    pcl::transformPointCloud(surfaces.strong.cloud, surfaces.strong.cloud, T);
    surfaces.strong.ClearKDTree();
  }
  if (surfaces.weak.cloud.size() > 0) {
    pcl::transformPointCloud(surfaces.weak.cloud, surfaces.weak.cloud, T);
    surfaces.weak.ClearKDTree();
  }
}

void LoamPointCloud::Save(const std::string& output_path,
                          bool combine_features) {
  if (boost::filesystem::exists(output_path)) {
    BEAM_INFO("Saving Loam point cloud to: {}", output_path);
  } else {
    BEAM_ERROR("File path ({}) does not exist, not saving loam cloud.",
               output_path);
    return;
  }

  if (combine_features) {
    PointCloud cloud_combined = edges.strong.cloud + edges.weak.cloud +
                                surfaces.strong.cloud + surfaces.weak.cloud;
    if (cloud_combined.size() == 0) {
      BEAM_WARN("Loam cloud empty. Not saving cloud.");
      return;
    }
    pcl::io::savePCDFileASCII(output_path + "combined_features.pcd",
                              cloud_combined);
  }

  if (edges.strong.cloud.size() == 0) {
    BEAM_WARN("Strong edge features are emtpy. Not saving cloud.");
  } else {
    pcl::io::savePCDFileASCII(output_path + "edge_features_strong.pcd",
                              edges.strong.cloud);
  }
  if (edges.weak.cloud.size() == 0) {
    BEAM_WARN("Weak edge features are emtpy. Not saving cloud.");
  } else {
    pcl::io::savePCDFileASCII(output_path + "edge_features_weak.pcd",
                              edges.weak.cloud);
  }
  if (surfaces.strong.cloud.size() == 0) {
    BEAM_WARN("Strong surface features are emtpy. Not saving cloud.");
  } else {
    pcl::io::savePCDFileASCII(output_path + "surface_features_strong.pcd",
                              surfaces.strong.cloud);
  }

  if (surfaces.weak.cloud.size() == 0) {
    BEAM_WARN("Weak surface features are emtpy. Not saving cloud.");
  } else {
    pcl::io::savePCDFileASCII(output_path + "surface_features_weak.pcd",
                              surfaces.weak.cloud);
  }
}

void LoamPointCloud::Merge(const LoamPointCloud& cloud){
  edges.strong.cloud += cloud.edges.strong.cloud;
  edges.strong.ClearKDTree();

  edges.weak.cloud += cloud.edges.weak.cloud;
  edges.weak.ClearKDTree();

  surfaces.strong.cloud += cloud.surfaces.strong.cloud;
  surfaces.strong.ClearKDTree();

  surfaces.weak.cloud += cloud.surfaces.weak.cloud;
  surfaces.weak.ClearKDTree();
}

} // namespace beam_matching
