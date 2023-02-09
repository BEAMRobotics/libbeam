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

#define PCL_NO_PRECOMPILE

#include <beam_utils/kdtree.h>
#include <beam_utils/pointclouds.h>

/** @brief Point label options. */
enum PointLabel {
  CORNER_SHARP = 255,      // sharp corner point
  CORNER_LESS_SHARP = 170, // less sharp corner point
  SURFACE_LESS_FLAT = 85, // less flat surface point
  SURFACE_FLAT = 0      // flat surface point
};


struct PointLoam {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  std::uint16_t ring;
  float time;
  std::int8_t type; // see PointLabel
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointLoam, (float, x, x)(float, y, y)(float, z, z)
    (float, intensity, intensity)(std::uint16_t, ring, ring)(float, time, time)
    (std::int8_t, type, type))
// clang-format on

namespace beam_matching {
/** @addtogroup matching
 *  @{ */

using LoamPointCloudCombined = pcl::PointCloud<PointLoam>;

/**
 * @brief struct for containing all data stored in a loam feature cloud (e.g.
 * sharp features)
 */
class LoamFeatureCloud {
public:
  /** Pointcloud containing xyz coordinates of all features */
  PointCloudIRT cloud;

  /** KD search tree for fast searching. Will only be built when BuildKDTree is
   * called. This will get cleared whenever TransformPointCloud is called as it
   * would need to be recalculated. */
  std::shared_ptr<beam::KdTree<PointXYZIRT>> kdtree{
      std::make_shared<beam::KdTree<PointXYZIRT>>(PointCloudIRT())};

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
  LoamPointCloud(const PointCloudIRT& edge_features_strong,
                 const PointCloudIRT& surface_features_strong,
                 const PointCloudIRT& edge_features_weak = PointCloudIRT(),
                 const PointCloudIRT& surface_features_weak = PointCloudIRT());

  /**
   * @brief load this loam pointcloud from a combined loam cloud. See
   * GetCombinedCloud
   * @param cloud
   */
  void LoadFromCombined(const LoamPointCloudCombined& cloud);

  /**
   * @brief Add a new set of strong surface features
   * @param new_features strong surface features
   * @param T if specified, new features will be transformed using T before
   * added to the cloud.
   */
  void AddSurfaceFeaturesStrong(
      const PointCloudIRT& new_features,
      const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity());

  /**
   * @brief Add a new set of weak (less flat) surface features
   * @param new_features weak surface features
   * @param T if specified, new features will be transformed using T before
   * added to the cloud.
   */
  void AddSurfaceFeaturesWeak(
      const PointCloudIRT& new_features,
      const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity());

  /**
   * @brief Add a new set of strong edge features
   * @param new_features strong edge features
   * @param T if specified, new features will be transformed using T before
   * added to the cloud.
   */
  void AddEdgeFeaturesStrong(
      const PointCloudIRT& new_features,
      const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity());

  /**
   * @brief Add a new set of weak (less sharp) edge features
   * @param new_features weak edge features
   * @param T if specified, new features will be transformed using T before
   * added to the cloud.
   */
  void AddEdgeFeaturesWeak(
      const PointCloudIRT& new_features,
      const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity());

  /**
   * @brief transforms a loam point cloud including all feature clouds (strong
   * and weak, edge and planar)
   * @param T transform to apply to all features [R, t; 0 1]
   */
  void TransformPointCloud(const Eigen::Matrix4d& T);

  /**
   * @brief method for saving a loam pointcloud. It will output 4 separate
   * clouds if all 4 features clouds are specified. If colors are not specified,
   * all will be output as white.
   * @param output_path full path to output directory which must already exist
   * @param combine_features optionally specify if you want to combine all
   * features into a single cloud in addition to all feature clouds.
   * @param r red color
   * @param g green color
   * @param b blue color
   */
  void Save(const std::string& output_path, bool combine_features = false,
            uint8_t r = 255, uint8_t g = 255, uint8_t b = 255) const;

  /**
   * @brief add a new loam pointcloud to this cloud. This will also clear all kd
   * search trees.
   * @param cloud new cloud to add
   */
  void Merge(const LoamPointCloud& cloud);

  /**
   * @brief print details of this cloud
   * @param stream input stream
   */
  void Print(std::ostream& stream = std::cout) const;

  /**
   * @brief get the total number of features
   * @return size of edges + size of surfaces (both strong and weak)
   */
  uint64_t Size() const;

  /**
   * @brief determine if all clouds are empty. This is more efficient than using
   * Size() != 0 or Size() > 0;
   * @return true if all 4 feature clouds are empty, false otherwise.
   */
  bool Empty() const;

  LoamPointCloud Copy() const {
    return LoamPointCloud(edges.strong.cloud, surfaces.strong.cloud,
                          edges.weak.cloud, surfaces.weak.cloud);
  }

  /**
   * @brief Get the Combined Cloud object
   *
   * @return LoamPointCloudCombined
   */
  LoamPointCloudCombined GetCombinedCloud() const;

  /** Edge (or sharp) features are directly accessible for ease of use */
  LoamFeatures edges;

  /** Surface (or planar or flat) features are directly accessible for ease of
   * use */
  LoamFeatures surfaces;
};

using LoamPointCloudPtr = std::shared_ptr<LoamPointCloud>;

/** @} group matching */
} // namespace beam_matching
