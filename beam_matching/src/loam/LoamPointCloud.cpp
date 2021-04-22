#include <beam_matching/LoamPointCloud.h>

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>

#include <beam_utils/log.h>

namespace beam_matching {

LoamPointCloud::LoamPointCloud(const PointCloud& edge_features,
                               const PointCloud& planar_features)
    : edge_features_(edge_features), planar_features_(planar_features) {}

void LoamPointCloud::AddPlanarFeatures(const PointCloud& new_features,
                                       const Eigen::Matrix4d& T) {
  if (!T.isIdentity()) {
    PointCloud new_features_transformed;
    pcl::transformPointCloud(new_features, new_features_transformed, T);
    planar_features_ += new_features_transformed;
    return;
  }
  planar_features_ += new_features;
  return;
}

void LoamPointCloud::AddEdgeFeatures(const PointCloud& new_features,
                                     const Eigen::Matrix4d& T) {
  if (!T.isIdentity()) {
    PointCloud new_features_transformed;
    pcl::transformPointCloud(new_features, new_features_transformed, T);
    edge_features_ += new_features_transformed;
    return;
  }
  edge_features_ += new_features;
  return;
}

PointCloud LoamPointCloud::EdgeFeatures() {
  return edge_features_;
}

PointCloud LoamPointCloud::PlanarFeatures() {
  return planar_features_;
}

void LoamPointCloud::TransformPointCloud(const Eigen::Matrix4d& T){
  pcl::transformPointCloud(edge_features_, edge_features_, T);
  pcl::transformPointCloud(planar_features_, planar_features_, T);
}

} // namespace beam_matching
