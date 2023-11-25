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

LoamPointCloud::LoamPointCloud(const LoamPointCloud& cloud,
                               const Eigen::Matrix4d& T) {
  Eigen::Affine3d TA(T);
  for (const auto& p : cloud.edges.strong.cloud) {
    edges.strong.cloud.push_back(pcl::transformPoint(p, TA));
  }
  for (const auto& p : cloud.edges.weak.cloud) {
    edges.weak.cloud.push_back(pcl::transformPoint(p, TA));
  }
  for (const auto& p : cloud.surfaces.strong.cloud) {
    surfaces.strong.cloud.push_back(pcl::transformPoint(p, TA));
  }
  for (const auto& p : cloud.surfaces.weak.cloud) {
    surfaces.weak.cloud.push_back(pcl::transformPoint(p, TA));
  }
  edges.strong.curvatures = cloud.edges.strong.curvatures;
  edges.weak.curvatures = cloud.edges.weak.curvatures;
  surfaces.strong.curvatures = cloud.surfaces.strong.curvatures;
  surfaces.weak.curvatures = cloud.surfaces.weak.curvatures;
}

void LoamFeatureCloud::BuildKDTree(bool override_tree) {
  // if tree is not empty, and we do not want to override, then do nothing
  if (!kdtree_empty && !override_tree) { return; }

  kdtree = std::make_shared<beam::KdTree<PointXYZIRT>>(cloud);
  kdtree_empty = false;
}

LoamPointCloud::LoamPointCloud(
    const PointCloudIRT& edge_features_strong,
    const PointCloudIRT& surface_features_strong,
    const PointCloudIRT& edge_features_weak,
    const PointCloudIRT& surface_features_weak,
    const std::vector<float>& edge_curvatures_strong,
    const std::vector<float>& surface_curvatures_strong,
    const std::vector<float>& edge_curvatures_weak,
    const std::vector<float>& surface_curvatures_weak) {
  edges.strong.cloud = edge_features_strong;
  surfaces.strong.cloud = surface_features_strong;
  edges.weak.cloud = edge_features_weak;
  surfaces.weak.cloud = surface_features_weak;
  edges.strong.curvatures = edge_curvatures_strong;
  edges.weak.curvatures = edge_curvatures_weak;
  surfaces.strong.curvatures = surface_curvatures_strong;
  surfaces.weak.curvatures = surface_curvatures_weak;
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

void LoamPointCloud::SaveCombined(const std::string& output_path,
                                  const std::string& filename, uint8_t r,
                                  uint8_t g, uint8_t b, bool verbose) const {
  if (!boost::filesystem::exists(output_path)) {
    BEAM_ERROR("File path ({}) does not exist, not saving loam cloud.",
               output_path);
    return;
  }

  LoamPointCloudCombined cloud_combined = GetCombinedCloud();
  if (cloud_combined.size() == 0) {
    BEAM_WARN("Loam cloud empty. Not saving cloud.");
    return;
  }

  std::string output_final = beam::CombinePaths(output_path, filename);
  if (verbose) {
    BEAM_INFO("Saving Loam point cloud with {} points to: {}",
              cloud_combined.size(), output_final);
  }

  std::string error_message{};
  if (r == 255 && g == 255 && b == 255) {
    if (!beam::SavePointCloud<PointLoam>(output_final, cloud_combined,
                                         beam::PointCloudFileType::PCDBINARY,
                                         error_message)) {
      BEAM_ERROR("Unable to save loam cloud. Reason: {}", error_message);
    }
    return;
  }

  // else color cloud
  pcl::PointCloud<PointLoamColored> colored;
  for (const PointLoam& p : cloud_combined) {
    PointLoamColored pc;
    pc.x = p.x;
    pc.y = p.y;
    pc.z = p.z;
    pc.intensity = p.intensity;
    pc.ring = p.ring;
    pc.time = p.time;
    pc.type = p.type;
    pc.curvature = p.curvature;
    pc.r = r;
    pc.g = g;
    pc.b = b;
    colored.push_back(pc);
  }
  if (!beam::SavePointCloud<PointLoamColored>(
          output_final, colored, beam::PointCloudFileType::PCDBINARY,
          error_message)) {
    BEAM_ERROR("Unable to save loam cloud. Reason: {}", error_message);
  }
}

void LoamPointCloud::Save(const std::string& output_path,
                          const std::string& prefix, uint8_t r, uint8_t g,
                          uint8_t b, bool verbose) const {
  if (!boost::filesystem::exists(output_path)) {
    BEAM_ERROR("File path ({}) does not exist, not saving loam cloud.",
               output_path);
    return;
  }

  std::string path_final = beam::CombinePaths(output_path, prefix);
  if (verbose) {
    BEAM_INFO("Saving Loam point clouds to: {}", path_final + "*");
  }

  std::string error_message{};
  // output all as white if colors not specified
  if (r == 255 && g == 255 && b == 255) {
    if (!beam::SavePointCloud<PointXYZIRT>(
            path_final + "edge_features_strong.pcd", edges.strong.cloud,
            beam::PointCloudFileType::PCDBINARY, error_message)) {
      BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
    }
    if (!beam::SavePointCloud<PointXYZIRT>(
            path_final + "edge_features_weak.pcd", edges.weak.cloud,
            beam::PointCloudFileType::PCDBINARY, error_message)) {
      BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
    }
    if (!beam::SavePointCloud<PointXYZIRT>(
            path_final + "surface_features_strong.pcd", surfaces.strong.cloud,
            beam::PointCloudFileType::PCDBINARY, error_message)) {
      BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
    }
    if (!beam::SavePointCloud<PointXYZIRT>(
            path_final + "surface_features_weak.pcd", surfaces.weak.cloud,
            beam::PointCloudFileType::PCDBINARY, error_message)) {
      BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
    }
    return;
  }

  PointCloudCol cloud_col;
  cloud_col = beam::ColorPointCloud(edges.strong.cloud, r, g, b);
  if (!beam::SavePointCloud<pcl::PointXYZRGB>(
          path_final + "edge_features_strong.pcd", cloud_col,
          beam::PointCloudFileType::PCDBINARY, error_message)) {
    BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
  }
  cloud_col = beam::ColorPointCloud(edges.weak.cloud, r, g, b);
  if (!beam::SavePointCloud<pcl::PointXYZRGB>(
          path_final + "edge_features_weak.pcd", cloud_col,
          beam::PointCloudFileType::PCDBINARY, error_message)) {
    BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
  }
  cloud_col = beam::ColorPointCloud(surfaces.strong.cloud, r, g, b);
  if (!beam::SavePointCloud<pcl::PointXYZRGB>(
          path_final + "surface_features_strong.pcd", cloud_col,
          beam::PointCloudFileType::PCDBINARY, error_message)) {
    BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
  }
  cloud_col = beam::ColorPointCloud(surfaces.weak.cloud, r, g, b);
  if (!beam::SavePointCloud<pcl::PointXYZRGB>(
          path_final + "surface_features_weak.pcd", cloud_col,
          beam::PointCloudFileType::PCDBINARY, error_message)) {
    BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
  }
}

void LoamPointCloud::Merge(const LoamPointCloud& cloud) {
  if (cloud.Size() == 0) { return; }

  edges.strong.cloud += cloud.edges.strong.cloud;
  edges.weak.cloud += cloud.edges.weak.cloud;
  surfaces.strong.cloud += cloud.surfaces.strong.cloud;
  surfaces.weak.cloud += cloud.surfaces.weak.cloud;

  edges.strong.ClearKDTree();
  edges.weak.ClearKDTree();
  surfaces.strong.ClearKDTree();
  surfaces.weak.ClearKDTree();

  for (float c : cloud.edges.strong.curvatures) {
    edges.strong.curvatures.push_back(c);
  }
  for (float c : cloud.edges.weak.curvatures) {
    edges.weak.curvatures.push_back(c);
  }
  for (float c : cloud.surfaces.strong.curvatures) {
    surfaces.strong.curvatures.push_back(c);
  }
  for (float c : cloud.surfaces.weak.curvatures) {
    surfaces.weak.curvatures.push_back(c);
  }
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

void LoamPointCloud::LoadFromCombined(const LoamPointCloudCombined& cloud) {
  for (const PointLoam& p : cloud) {
    PointXYZIRT pnew;
    pnew.x = p.x;
    pnew.y = p.y;
    pnew.z = p.z;
    pnew.intensity = p.intensity;
    pnew.ring = p.ring;
    pnew.time = p.time;
    if (p.type == PointLabel::CORNER_SHARP) {
      edges.strong.cloud.push_back(pnew);
      edges.strong.curvatures.push_back(p.curvature);
    } else if (p.type == PointLabel::CORNER_LESS_SHARP) {
      edges.weak.cloud.push_back(pnew);
      edges.weak.curvatures.push_back(p.curvature);
    } else if (p.type == PointLabel::SURFACE_FLAT) {
      surfaces.strong.cloud.push_back(pnew);
      surfaces.strong.curvatures.push_back(p.curvature);
    } else if (p.type == PointLabel::SURFACE_LESS_FLAT) {
      surfaces.weak.cloud.push_back(pnew);
      surfaces.weak.curvatures.push_back(p.curvature);
    } else {
      BEAM_ERROR("invalid type parameter in cloud.");
      throw std::runtime_error{"invalid type parameter in cloud"};
    }
  }
}

LoamPointCloudCombined LoamPointCloud::GetCombinedCloud() const {
  LoamPointCloudCombined cloud;
  AddPointsToLoamCloudCombined(cloud, edges.strong, PointLabel::CORNER_SHARP);
  AddPointsToLoamCloudCombined(cloud, edges.weak,
                               PointLabel::CORNER_LESS_SHARP);
  AddPointsToLoamCloudCombined(cloud, surfaces.strong,
                               PointLabel::SURFACE_FLAT);
  AddPointsToLoamCloudCombined(cloud, surfaces.weak,
                               PointLabel::SURFACE_LESS_FLAT);
  return cloud;
}

void LoamPointCloud::AddPointsToLoamCloudCombined(
    LoamPointCloudCombined& cloud_combined, const LoamFeatureCloud& features,
    PointLabel label) const {
  bool curvatures_exist =
      features.curvatures.size() == features.cloud.points.size();
  for (size_t i = 0; i < features.cloud.points.size(); i++) {
    const PointXYZIRT& p = features.cloud.points[i];
    PointLoam pnew;
    pnew.x = p.x;
    pnew.y = p.y;
    pnew.z = p.z;
    pnew.intensity = p.intensity;
    pnew.ring = p.ring;
    pnew.time = p.time;
    pnew.type = label;
    if (curvatures_exist) { pnew.type = features.curvatures[i]; }
    cloud_combined.push_back(pnew);
  }
}

} // namespace beam_matching
