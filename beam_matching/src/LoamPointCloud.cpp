#include <beam_matching/LoamPointCloud.h>

#include <Eigen/Geometry>

#include <beam_utils/log.h>

namespace beam_matching {

LoamPointCloud::LoamPointCloud(const PointCloud& edge_features,
                               const PointCloud& planar_features) {}

LoamPointCloud::LoamPointCloud(const PointCloud& raw_cloud,
                               const std::shared_ptr<LoamParams>& params);

void LoamPointCloud::AddPlanarFeatures(
    const PointCloud& new_features,
    const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity()) {
  return;
}

void LoamPointCloud::AddEdgeFeatures(
    const PointCloud& new_features,
    const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity()) {
  return;
}

void LoamPointCloud::ExtractLoamFeatures(const PointCloud& cloud,
                                         PointCloud& edge_features,
                                         PointCloud& planar_features) {
  return;
}

PointCloud LoamPointCloud::ExtractEdgeFeatures(const PointCloud& raw_cloud) {
  return PointCloud();
}

PointCloud LoamPointCloud::ExtractPlanarFeatures(const PointCloud& raw_cloud) {
  return PointCloud();
}

} // namespace beam_matching
