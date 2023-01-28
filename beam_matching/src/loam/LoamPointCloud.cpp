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
  kdtree->clear();
  kdtree_empty = true;
}

void LoamFeatureCloud::BuildKDTree(bool override_tree) {
  // if tree is not empty, and we do not want to override, then do nothing
  if (!kdtree_empty && !override_tree) { return; }

  kdtree = std::make_shared<beam::KdTree<PointXYZIRT>>(cloud);
  kdtree_empty = false;
}

LoamPointCloud::LoamPointCloud(const PointCloudIRT& edge_features_strong,
                               const PointCloudIRT& surface_features_strong,
                               const PointCloudIRT& edge_features_weak,
                               const PointCloudIRT& surface_features_weak) {
  edges.strong.cloud = edge_features_strong;
  surfaces.strong.cloud = surface_features_strong;
  edges.weak.cloud = edge_features_weak;
  surfaces.weak.cloud = surface_features_weak;
}

void LoamPointCloud::AddSurfaceFeaturesStrong(const PointCloudIRT& new_features,
                                              const Eigen::Matrix4d& T) {
  if (!T.isIdentity()) {
    PointCloudIRT new_features_transformed;
    pcl::transformPointCloud(new_features, new_features_transformed, T);
    surfaces.strong.cloud += new_features_transformed;
    return;
  }
  surfaces.strong.cloud += new_features;
  return;
}

void LoamPointCloud::AddEdgeFeaturesStrong(const PointCloudIRT& new_features,
                                           const Eigen::Matrix4d& T) {
  if (!T.isIdentity()) {
    PointCloudIRT new_features_transformed;
    pcl::transformPointCloud(new_features, new_features_transformed, T);
    edges.strong.cloud += new_features_transformed;
    return;
  }
  edges.strong.cloud += new_features;
  return;
}

void LoamPointCloud::AddSurfaceFeaturesWeak(const PointCloudIRT& new_features,
                                            const Eigen::Matrix4d& T) {
  if (!T.isIdentity()) {
    PointCloudIRT new_features_transformed;
    pcl::transformPointCloud(new_features, new_features_transformed, T);
    surfaces.weak.cloud += new_features_transformed;
    return;
  }
  surfaces.weak.cloud += new_features;
  return;
}

void LoamPointCloud::AddEdgeFeaturesWeak(const PointCloudIRT& new_features,
                                         const Eigen::Matrix4d& T) {
  if (!T.isIdentity()) {
    PointCloudIRT new_features_transformed;
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

void LoamPointCloud::Save(const std::string& output_path, bool combine_features,
                          uint8_t r, uint8_t g, uint8_t b) const {
  if (boost::filesystem::exists(output_path)) {
    BEAM_INFO("Saving Loam point cloud to: {}", output_path);
  } else {
    BEAM_ERROR("File path ({}) does not exist, not saving loam cloud.",
               output_path);
    return;
  }

  PointCloudIRT cloud_combined;
  if (combine_features) {
    cloud_combined = edges.strong.cloud;
    cloud_combined += edges.weak.cloud;
    cloud_combined += surfaces.strong.cloud;
    cloud_combined += surfaces.weak.cloud;
    if (cloud_combined.size() == 0) {
      BEAM_WARN("Loam cloud empty. Not saving cloud.");
      return;
    }
  }

  // output all as white if colors not specified
  if (r == 255 && g == 255 && b == 255) {
    std::string error_message{};
    if (combine_features) {
      if (!beam::SavePointCloud<PointXYZIRT>(
              output_path + "combined_features.pcd", cloud_combined,
              beam::PointCloudFileType::PCDBINARY, error_message)) {
        BEAM_ERROR("Unable to save loam cloud. Reason: {}", error_message);
      }
    }

    if (!beam::SavePointCloud<PointXYZIRT>(
            output_path + "edge_features_strong.pcd", edges.strong.cloud,
            beam::PointCloudFileType::PCDBINARY, error_message)) {
      BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
    }
    if (!beam::SavePointCloud<PointXYZIRT>(
            output_path + "edge_features_weak.pcd", edges.weak.cloud,
            beam::PointCloudFileType::PCDBINARY, error_message)) {
      BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
    }
    if (!beam::SavePointCloud<PointXYZIRT>(
            output_path + "surface_features_strong.pcd", surfaces.strong.cloud,
            beam::PointCloudFileType::PCDBINARY, error_message)) {
      BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
    }
    if (!beam::SavePointCloud<PointXYZIRT>(
            output_path + "surface_features_weak.pcd", surfaces.weak.cloud,
            beam::PointCloudFileType::PCDBINARY, error_message)) {
      BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
    }
  } else {
    PointCloudCol cloud_col;
    std::string error_message{};
    if (combine_features) {
      cloud_col = beam::ColorPointCloud(cloud_combined, r, g, b);
      if (!beam::SavePointCloud<pcl::PointXYZRGB>(
              output_path + "combined_features.pcd", cloud_col,
              beam::PointCloudFileType::PCDBINARY, error_message)) {
        BEAM_ERROR("Unable to save loam cloud. Reason: {}", error_message);
      }
    }

    cloud_col = beam::ColorPointCloud(edges.strong.cloud, r, g, b);
    if (!beam::SavePointCloud<pcl::PointXYZRGB>(
            output_path + "edge_features_strong.pcd", cloud_col,
            beam::PointCloudFileType::PCDBINARY, error_message)) {
      BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
    }
    cloud_col = beam::ColorPointCloud(edges.weak.cloud, r, g, b);
    if (!beam::SavePointCloud<pcl::PointXYZRGB>(
            output_path + "edge_features_weak.pcd", cloud_col,
            beam::PointCloudFileType::PCDBINARY, error_message)) {
      BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
    }
    cloud_col = beam::ColorPointCloud(surfaces.strong.cloud, r, g, b);
    if (!beam::SavePointCloud<pcl::PointXYZRGB>(
            output_path + "surface_features_strong.pcd", cloud_col,
            beam::PointCloudFileType::PCDBINARY, error_message)) {
      BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
    }
    cloud_col = beam::ColorPointCloud(surfaces.weak.cloud, r, g, b);
    if (!beam::SavePointCloud<pcl::PointXYZRGB>(
            output_path + "surface_features_weak.pcd", cloud_col,
            beam::PointCloudFileType::PCDBINARY, error_message)) {
      BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
    }
  }
}

void LoamPointCloud::Merge(const LoamPointCloud& cloud) {
  if (cloud.Size() == 0) { return; }

  edges.strong.cloud += cloud.edges.strong.cloud;
  edges.strong.ClearKDTree();

  edges.weak.cloud += cloud.edges.weak.cloud;
  edges.weak.ClearKDTree();

  surfaces.strong.cloud += cloud.surfaces.strong.cloud;
  surfaces.strong.ClearKDTree();

  surfaces.weak.cloud += cloud.surfaces.weak.cloud;
  surfaces.weak.ClearKDTree();
}

void LoamPointCloud::Print(std::ostream& stream) const {
  stream << "Feature |  Type  | KdTree Empty | No. of Points   \n"
         << " Edges  | strong | " << edges.strong.kdtree_empty << "          | "
         << edges.strong.cloud.size() << "\n"
         << " Edges  |  weak  | " << edges.weak.kdtree_empty << "          | "
         << edges.weak.cloud.size() << "\n"
         << "Surfaces| strong | " << surfaces.strong.kdtree_empty
         << "          | " << surfaces.strong.cloud.size() << "\n"
         << "Surfaces|  weak  | " << surfaces.weak.kdtree_empty
         << "          | " << surfaces.weak.cloud.size() << "\n";
}

uint64_t LoamPointCloud::Size() const {
  return edges.strong.cloud.size() + edges.weak.cloud.size() +
         surfaces.strong.cloud.size() + surfaces.weak.cloud.size();
}

bool LoamPointCloud::Empty() const {
  if (!edges.strong.cloud.empty()) { return false; }
  if (!edges.weak.cloud.empty()) { return false; }
  if (!surfaces.strong.cloud.empty()) { return false; }
  if (!surfaces.weak.cloud.empty()) { return false; }
  return true;
}

LoamPointCloudCombined LoamPointCloud::GetCombinedCloud() {
  LoamPointCloudCombined cloud;
  for (const PointXYZIRT& p : edges.strong.cloud.points) {
    PointLoam pnew;
    pnew.x = p.x;
    pnew.y = p.y;
    pnew.z = p.z;
    pnew.intensity = p.intensity;
    pnew.ring = p.ring;
    pnew.time = p.time;
    pnew.type = 2;
    cloud.push_back(pnew);
  }
  for (const PointXYZIRT& p : edges.weak.cloud.points) {
    PointLoam pnew;
    pnew.x = p.x;
    pnew.y = p.y;
    pnew.z = p.z;
    pnew.intensity = p.intensity;
    pnew.ring = p.ring;
    pnew.time = p.time;
    pnew.type = 1;
    cloud.push_back(pnew);
  }
  for (const PointXYZIRT& p : surfaces.strong.cloud.points) {
    PointLoam pnew;
    pnew.x = p.x;
    pnew.y = p.y;
    pnew.z = p.z;
    pnew.intensity = p.intensity;
    pnew.ring = p.ring;
    pnew.time = p.time;
    pnew.type = 0;
    cloud.push_back(pnew);
  }
  for (const PointXYZIRT& p : surfaces.weak.cloud.points) {
    PointLoam pnew;
    pnew.x = p.x;
    pnew.y = p.y;
    pnew.z = p.z;
    pnew.intensity = p.intensity;
    pnew.ring = p.ring;
    pnew.time = p.time;
    pnew.type = -1;
    cloud.push_back(pnew);
  }
  return cloud;
}

} // namespace beam_matching
