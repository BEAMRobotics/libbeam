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

namespace beam_matching {
/** @addtogroup matching
 *  @{ */

class LoamPointCloud {
public:
  /**
   * @brief Default constructor
   */
  LoamPointCloud() = default;

  /**
   * @brief Constructor that takes in two pointclouds that are already separated
   * into edge and planar clouds
   * @param edge_features
   * @param planar_features
   */
  LoamPointCloud(const PointCloud& edge_features,
                 const PointCloud& planar_features);

  /**
   * @brief Add a new set of planar features
   * @param new_features planar features
   * @param T if specified, new features will be transformed using T before
   * added to the cloud.
   */
  void
      AddPlanarFeatures(const PointCloud& new_features,
                        const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity());

  /**
   * @brief Add a new set of edge features
   * @param new_features edge features
   * @param T if specified, new features will be transformed using T before
   * added to the cloud.
   */
  void AddEdgeFeatures(const PointCloud& new_features,
                       const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity());

  PointCloud EdgeFeatures();

  PointCloud PlanarFeatures();

  void TransformPointCloud(const Eigen::Matrix4d& T);

  private:
  PointCloud edge_features_;
  PointCloud planar_features_;
}

using LoamPointCloudPtr = boost::shared_ptr<LoamPointCloud>;

/** @} group matching */
} // namespace beam_matching
