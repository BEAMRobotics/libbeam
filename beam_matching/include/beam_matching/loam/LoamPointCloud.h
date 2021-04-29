/** @file
 * @ingroup matching
 *
 * The is an implementation of Lidar Odometry and Mapping (LOAM). See the
 * following papers:
 *
 *    Zhang, J., & Singh, S. (2014). LOAM : Lidar Odometry and Mapping in
 * Real-time. Robotics: Science and Systems.
 * https://doi.org/10.1007/s10514-016-9548-2
 *
 *    Zhang, J., & Singh, S. (2018). Laser–visual–inertial odometry and mapping
 * with high robustness and low drift. Journal of Field Robotics, 35(8),
 * 1242–1264. https://doi.org/10.1002/rob.21809
 *
 * This code was derived from the following repos:
 *
 *    https://github.com/laboshinl/loam_velodyne
 *
 *    https://github.com/libing64/lidar_pose_estimator
 *
 *    https://github.com/TixiaoShan/LIO-SAM
 *
 */

#pragma once

#include <beam_matching/Matcher.h>
#include <beam_utils/pointclouds.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace beam_matching {
/** @addtogroup matching
 *  @{ */

struct LoamFeatureCloud {
  PointCloud cloud;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  void BuildKDTree(bool override_tree = false);
  void Clear();
  void ClearKDTree();
  bool kdtree_empty{true};
};

struct LoamFeatures {
  LoamFeatureCloud strong;
  LoamFeatureCloud weak;
  void Clear();
};

class LoamPointCloud {
public:
  /**
   * @brief Default constructor
   */
  LoamPointCloud() = default;

  /**
   * @brief Constructor that takes in two pointclouds that are already separated
   * into edge and surface clouds
   * @param edge_features
   * @param surface_features
   * @param edge_features_less_sharp
   * @param surface_features_less_flat
   */
  LoamPointCloud(const PointCloud& edge_features_strong,
                 const PointCloud& surface_features_strong,
                 const PointCloud& edge_features_weak = PointCloud(),
                 const PointCloud& surface_features_weak = PointCloud());

  /**
   * @brief Add a new set of surface features
   * @param new_features surface features
   * @param T if specified, new features will be transformed using T before
   * added to the cloud.
   */
  void AddSurfaceFeaturesStrong(
      const PointCloud& new_features,
      const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity());

  /**
   * @brief Add a new set of weak (less flat) surface features
   * @param new_features surface features
   * @param T if specified, new features will be transformed using T before
   * added to the cloud.
   */
  void AddSurfaceFeaturesWeak(
      const PointCloud& new_features,
      const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity());

  /**
   * @brief Add a new set of edge features
   * @param new_features edge features
   * @param T if specified, new features will be transformed using T before
   * added to the cloud.
   */
  void AddEdgeFeaturesStrong(
      const PointCloud& new_features,
      const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity());

  /**
   * @brief Add a new set of weak (less sharp) edge features
   * @param new_features edge features
   * @param T if specified, new features will be transformed using T before
   * added to the cloud.
   */
  void AddEdgeFeaturesWeak(
      const PointCloud& new_features,
      const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity());

  void TransformPointCloud(const Eigen::Matrix4d& T);

  void Save(const std::string& output_path);

  LoamFeatures edges;
  LoamFeatures surfaces;
};

using LoamPointCloudPtr = std::shared_ptr<LoamPointCloud>;

/** @} group matching */
} // namespace beam_matching
