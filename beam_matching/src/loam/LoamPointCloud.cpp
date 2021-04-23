#include <beam_matching/loam/LoamPointCloud.h>

#include <Eigen/Geometry>
#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <beam_utils/log.h>

namespace beam_matching {

LoamPointCloud::LoamPointCloud(const PointCloud& edge_features,
                               const PointCloud& planar_features,
                               const PointCloud& edge_features_less_sharp,
                               const PointCloud& planar_features_less_flat)
    : edge_features_(edge_features),
      planar_features_(planar_features),
      edge_features_less_sharp_(edge_features_less_sharp),
      planar_features_less_flat_(planar_features_less_flat) {}

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

void LoamPointCloud::AddPlanarFeaturesLessFlat(const PointCloud& new_features,
                                               const Eigen::Matrix4d& T) {
  if (!T.isIdentity()) {
    PointCloud new_features_transformed;
    pcl::transformPointCloud(new_features, new_features_transformed, T);
    planar_features_less_flat_ += new_features_transformed;
    return;
  }
  planar_features_less_flat_ += new_features;
  return;
}

void LoamPointCloud::AddEdgeFeaturesLessSharp(const PointCloud& new_features,
                                              const Eigen::Matrix4d& T) {
  if (!T.isIdentity()) {
    PointCloud new_features_transformed;
    pcl::transformPointCloud(new_features, new_features_transformed, T);
    edge_features_less_sharp_ += new_features_transformed;
    return;
  }
  edge_features_less_sharp_ += new_features;
  return;
}

PointCloud LoamPointCloud::EdgeFeatures() {
  return edge_features_;
}

PointCloud LoamPointCloud::PlanarFeatures() {
  return planar_features_;
}

PointCloud LoamPointCloud::EdgeFeaturesLessSharp() {
  return edge_features_;
}

PointCloud LoamPointCloud::PlanarFeaturesLessFlat() {
  return planar_features_;
}

void LoamPointCloud::TransformPointCloud(const Eigen::Matrix4d& T) {
  pcl::transformPointCloud(edge_features_, edge_features_, T);
  pcl::transformPointCloud(planar_features_, planar_features_, T);
  pcl::transformPointCloud(edge_features_less_sharp_, edge_features_less_sharp_,
                           T);
  pcl::transformPointCloud(planar_features_less_flat_,
                           planar_features_less_flat_, T);
}

void LoamPointCloud::Save(const std::string& output_path) {
  if (boost::filesystem::exists(output_path)) {
    BEAM_INFO("Saving Loam point cloud to: {}", output_path);
  } else {
    BEAM_ERROR("File path ({}) does not exist, not saving loam cloud.",
               output_path);
    return;
  }

  if (edge_features_.size() == 0) {
    BEAM_WARN("Edge features are emtpy. Not saving cloud.");
  } else {
    pcl::io::savePCDFileASCII(output_path + "edge_features.pcd",
                              edge_features_);
  }
  if (edge_features_less_sharp_.size() == 0) {
    BEAM_WARN("Less sharp edge features are emtpy. Not saving cloud.");
  } else {
    pcl::io::savePCDFileASCII(output_path + "edge_features_less_sharp.pcd",
                              edge_features_less_sharp_);
  }
  if (planar_features_.size() == 0) {
    BEAM_WARN("Planar features are emtpy. Not saving cloud.");
  } else {
    pcl::io::savePCDFileASCII(output_path + "planar_features.pcd",
                              planar_features_);
  }

  if (planar_features_less_flat_.size() == 0) {
    BEAM_WARN("Less flat planar features are emtpy. Not saving cloud.");
  } else {
    pcl::io::savePCDFileASCII(output_path + "planar_features_less_flat.pcd",
                              planar_features_less_flat_);
  }
}
} // namespace beam_matching
