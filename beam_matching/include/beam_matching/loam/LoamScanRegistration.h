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

#include <beam_matching/loam/LoamParams.h>
#include <beam_matching/loam/LoamPointCloud.h>

namespace beam_matching {
/** @addtogroup matching
 *  @{ */

class LoamScanRegistration {
public:
  /**
   * @brief constructor
   * @param params required parameters
   */
  LoamScanRegistration(const LoamParamsPtr& params);

  /**
   * @brief destructor
   */
  ~LoamScanRegistration();

  /**
   * @brief performs the loam scan registration
   * @param ref reference pointcloud (cloud we are registering against). This
   * can be a single scan or a full map
   * @param tgt target pointcloud (cloud whose pose we want w.r.t. the
   * reference). This can be a single scan or a full map
   * @param T_REF_TGT optional inital transform between the two clouds
   * @return true if registration was successful
   */
  bool RegisterScans(
      const LoamPointCloudPtr& ref, const LoamPointCloudPtr& tgt,
      const Eigen::Matrix4d& T_REF_TGT = Eigen::Matrix4d::Identity());

  /**
   * @brief get result
   * @return transform from target cloud to reference cloud
   */
  Eigen::Matrix4d GetT_REF_TGT();

private:
  void Setup();

  bool GetEdgeMeasurements();

  bool GetSurfaceMeasurements();

  bool Solve(int iteration);

  bool HasConverged(int iteration);

  void OutputResults(int iteration);

  struct EdgeMeasurement {
    Eigen::Vector3d query_pt;
    Eigen::Vector3d ref_pt1;
    Eigen::Vector3d ref_pt2;
  };

  struct SurfaceMeasurement {
    Eigen::Vector3d query_pt;
    Eigen::Vector3d ref_pt1;
    Eigen::Vector3d ref_pt2;
    Eigen::Vector3d ref_pt3;
  };

  LoamParamsPtr params_;

  LoamPointCloudPtr ref_;
  LoamPointCloudPtr tgt_;
  std::vector<EdgeMeasurement> edge_measurements_;
  std::vector<SurfaceMeasurement> surface_measurements_;
  bool registration_successful_{true};
  bool converged_{false};
  Eigen::Matrix4d T_REF_TGT_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d T_REF_TGT_prev_iter_{Eigen::Matrix4d::Identity()};

  // Debugging tools
  std::string debug_output_path_{"/home/nick/tmp/loam_tests/"};
  bool output_results_{false};

};

/** @} group matching */
} // namespace beam_matching
