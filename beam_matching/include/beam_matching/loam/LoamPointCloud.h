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

/**
 * @brief struct for containing all data stored in a loam feature cloud (e.g.
 * sharp features)
 */
struct LoamFeatureCloud {
  /** Pointcloud containing xyz coordinates of all features */
  PointCloud cloud;

  /** KD search tree for fast searching. Will only be built when BuildKDTree is
   * called. This will get cleared whenever TransformPointCloud is called as it
   * would need to be recalculated. */
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  /** Builds the KD search tree and sets the kdtree_empty to false */
  void BuildKDTree(bool override_tree = false);

  /** Clears the KD tree and cloud */
  void Clear();

  /** Clears the KD tree */
  void ClearKDTree();

  /** Bool to determine if kdtree is built or not. Helps us make sure we don't
   * keep rebuilding a tree that's already built because this takes time. */
  bool kdtree_empty{true};
};

/**
 * @brief struct for containing strong and weak features
 */
struct LoamFeatures {
  /** Strong features (sharp edge features, or flat surface features) */
  LoamFeatureCloud strong;

  /** Weak features (less sharp edge features, or less flat surface features) */
  LoamFeatureCloud weak;

  /** Calls clear on in LoamFeatureCloud for both weak and strong features. */
  void Clear();
};

/**
 * @brief class containing all loam features, as well as useful functions for
 * adding, clearing and transforming clouds, as well as building or clearing KD
 * search trees
 */
class LoamPointCloud {
public:
  /**
   * @brief Default constructor
   */
  LoamPointCloud() = default;

  /**
   * @brief Constructor that takes in 2 or 4 point clouds of features (weak and
   * strong edge and planar features)
   * @param edge_features_strong required strong edge (sharp) features
   * @param surface_features_strong required strong surface (flat or planar)
   * features
   * @param edge_features_weak defaults to zero (weak features not required)
   * @param surface_features_weak defaults to zero (weak features not required)
   */
  LoamPointCloud(const PointCloud& edge_features_strong,
                 const PointCloud& surface_features_strong,
                 const PointCloud& edge_features_weak = PointCloud(),
                 const PointCloud& surface_features_weak = PointCloud());

  /**
   * @brief Add a new set of strong surface features
   * @param new_features strong surface features
   * @param T if specified, new features will be transformed using T before
   * added to the cloud.
   */
  void AddSurfaceFeaturesStrong(
      const PointCloud& new_features,
      const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity());

  /**
   * @brief Add a new set of weak (less flat) surface features
   * @param new_features weak surface features
   * @param T if specified, new features will be transformed using T before
   * added to the cloud.
   */
  void AddSurfaceFeaturesWeak(
      const PointCloud& new_features,
      const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity());

  /**
   * @brief Add a new set of strong edge features
   * @param new_features strong edge features
   * @param T if specified, new features will be transformed using T before
   * added to the cloud.
   */
  void AddEdgeFeaturesStrong(
      const PointCloud& new_features,
      const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity());

  /**
   * @brief Add a new set of weak (less sharp) edge features
   * @param new_features weak edge features
   * @param T if specified, new features will be transformed using T before
   * added to the cloud.
   */
  void AddEdgeFeaturesWeak(
      const PointCloud& new_features,
      const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity());

  /**
   * @brief transforms a loam point cloud including all feature clouds (strong
   * and weak, edge and planar)
   * @param T transform to apply to all features [R, t; 0 1]
   */
  void TransformPointCloud(const Eigen::Matrix4d& T);

  /**
   * @brief method for saving a loam pointcloud. It will output 4 separate
   * clouds if all 4 features clouds are specified.
   * @param output_path full path to output directory which must already exist
   * @param combine_features optionally specify if you want to combine all
   * features into a single cloud in addition to all feature clouds.
   */
  void Save(const std::string& output_path, bool combine_features = false);

  /**
   * @brief add a new loam pointcloud to this cloud. This will also clear all kd search trees.
   * @param cloud new cloud to add
   */
  void Merge(const LoamPointCloud& cloud);

  /** 
   * @brief print details of this cloud
   * @param stream input stream
   */
  void Print(std::ostream& stream = std::cout) const;

  /** Edge (or sharp) features are directly accessible for ease of use */
  LoamFeatures edges;

  /** Surface (or planar or flat) features are directly accessible for ease of
   * use */
  LoamFeatures surfaces;
};

using LoamPointCloudPtr = std::shared_ptr<LoamPointCloud>;

/** @} group matching */
} // namespace beam_matching
