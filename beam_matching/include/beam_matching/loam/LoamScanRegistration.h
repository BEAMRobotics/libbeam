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

/**
 * @brief class for performing registration of two loam clouds
 */
class LoamScanRegistration {
public:
  /**
   * @brief constructor using params
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

  /**
   * @brief Function for saving results. Stores the results as 3
   * separate clouds:
   *
   *  (1) reference cloud (blue points)
   *
   *  (2) target cloud initial: target cloud transformed into the reference
   *  cloud frame using the initial guess of it's relative pose. Since this
   *  matcher class doesn't allow you to provide an initial guess, the target
   *  cloud should already be in the reference frame using some guess.
   *
   *  (3) target cloud refined: target cloud transformed into the reference
   *  cloud frame using the refined pose (or transform calculated herein)
   *
   */
  void SaveResults(const std::string& output_path, const std::string& prefix = "cloud") const;

private:
  void Setup();

  bool GetEdgeMeasurements();

  bool GetSurfaceMeasurements();

  bool Solve(int iteration);

  bool HasConverged(int iteration);

  void OutputResults(int iteration);

  /** Simple struct for storing an edge measurement which contains a query point
   * for the target cloud and two reference points from the reference cloud */
  struct EdgeMeasurement {
    Eigen::Vector3d query_pt;
    Eigen::Vector3d ref_pt1;
    Eigen::Vector3d ref_pt2;
  };

  /** Simple struct for storing a surface measurement which contains a query
   * point for the target cloud and three reference points from the reference
   * cloud */
  struct SurfaceMeasurement {
    Eigen::Vector3d query_pt;
    Eigen::Vector3d ref_pt1;
    Eigen::Vector3d ref_pt2;
    Eigen::Vector3d ref_pt3;
  };

  /** struct to hold optimization summary for each iteration */
  struct OptimizationSummary {
    int correspondence_iteration_number{0};
    int surface_measurements{0};
    int edge_measurements{0};
    double translation_change_m{0};
    double rotation_change_deg{0};
    std::string ceres_termination{};
    int ceres_residuals{0};
    int ceres_iterations{0};
    double ceres_initial_cost{0};
    double ceres_final_cost{0};
    std::string correspondence_termination{};

    void Print();
  };

  LoamParamsPtr params_;

  OptimizationSummary optimization_summary_;

  LoamPointCloudPtr ref_;
  LoamPointCloudPtr tgt_;
  std::vector<EdgeMeasurement> edge_measurements_;
  std::vector<SurfaceMeasurement> surface_measurements_;
  bool registration_successful_{true};
  bool converged_{false};
  Eigen::Matrix4d T_REF_TGT_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d T_REF_TGT_prev_iter_{Eigen::Matrix4d::Identity()};

  // Debugging tools
  std::string debug_output_path_{"/home/nick/debug/loam_tests/"};
  std::string debug_output_path_stamped_;

  /* outputs scans before and after registration. This is diffent than running
   * the OutputResults private function which outputs results for each
   * iteration. */
  bool output_results_{false};
};

/** @} group matching */
} // namespace beam_matching
