#include <beam_matching/loam/LoamScanRegistration.h>

#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <ceres/loss_function.h>
#include <ceres/solver.h>
#include <ceres/types.h>
#include <pcl/common/transforms.h>

#include <beam_optimization/PointToLineCost.h>
#include <beam_optimization/PointToPlaneCost.h>
#include <beam_utils/log.h>
#include <beam_utils/math.h>
#include <beam_utils/se3.h>

namespace beam_matching {

LoamScanRegistration::LoamScanRegistration(const LoamParamsPtr& params)
    : params_(params) {}

LoamScanRegistration::~LoamScanRegistration() {
  if (ref_) { ref_.reset(); }
  if (tgt_) { tgt_.reset(); }
  edge_measurements_.clear();
  surface_measurements_.clear();
}

bool LoamScanRegistration::RegisterScans(const LoamPointCloudPtr& ref,
                                         const LoamPointCloudPtr& tgt,
                                         const Eigen::Matrix4d& T_REF_TGT) {
  ref_ = ref;
  tgt_ = tgt;
  T_REF_TGT_ = T_REF_TGT;
  T_REF_TGT_prev_iter_ = T_REF_TGT_;

  Setup();

  if (output_results_) {
    std::string time_now =
        beam::ConvertTimeToDate(std::chrono::system_clock::now());
    debug_output_path_stamped_ =
        beam::CombinePaths(debug_output_path_, time_now);
    boost::filesystem::create_directory(debug_output_path_stamped_);
  }

  int iteration = 0;
  while (true) {
    if (!GetEdgeMeasurements()) {
      registration_successful_ = false;
      break;
    }

    if (!GetSurfaceMeasurements()) {
      registration_successful_ = false;
      break;
    }

    if (!Solve(iteration)) {
      registration_successful_ = false;
      break;
    }

    if (HasConverged(iteration)) {
      OutputResults(iteration);
      break;
    }
    OutputResults(iteration);
    T_REF_TGT_prev_iter_ = T_REF_TGT_;
    iteration++;
  }
  return registration_successful_;
}

Eigen::Matrix4d LoamScanRegistration::GetT_REF_TGT() {
  if (!registration_successful_) {
    BEAM_WARN("Registration unsuccessful, returning identity matrix");
    return Eigen::Matrix4d::Identity();
  }
  return T_REF_TGT_;
}

void LoamScanRegistration::Setup() {
  registration_successful_ = true;
  edge_measurements_.clear();
  surface_measurements_.clear();
}

bool LoamScanRegistration::GetEdgeMeasurements() {
  edge_measurements_.clear();

  // transform target cloud to reference frame with current estimate
  PointCloudIRT tgt_features;
  if (T_REF_TGT_.isIdentity()) {
    tgt_features = tgt_->edges.strong.cloud;
  } else {
    pcl::transformPointCloud(tgt_->edges.strong.cloud, tgt_features,
                             T_REF_TGT_);
  }

  // build kdtrees if not already done
  if (ref_->edges.strong.cloud.size() > 0) {
    ref_->edges.strong.BuildKDTree(false);
  } else {
    BEAM_ERROR("Reference cloud has no edge features, aborting registration.");
    return false;
  }

  for (size_t tgt_iter = 0; tgt_iter < tgt_features.size(); tgt_iter++) {
    auto& search_pt = tgt_features.points.at(tgt_iter);
    auto& query_pt = tgt_->edges.strong.cloud.at(tgt_iter);

    // search for correspondence in strong fetures
    std::vector<uint32_t> point_search_ind;
    std::vector<float> point_search_sq_dist;
    int num_returned = ref_->edges.strong.kdtree->nearestKSearch(
        search_pt, 2, point_search_ind, point_search_sq_dist);

    // if both correspondences are close enough, add measurement, else check in
    // weak features
    if (num_returned != 2) {
      // do nothing, we will check weak features
    } else if (point_search_sq_dist.at(0) <
                   params_->max_correspondence_distance &&
               point_search_sq_dist.at(1) <
                   params_->max_correspondence_distance) {
      EdgeMeasurement measurement;

      measurement.query_pt =
          Eigen::Vector3d(query_pt.x, query_pt.y, query_pt.z);

      auto& ref_pt1 =
          ref_->edges.strong.cloud.points.at(point_search_ind.at(0));
      measurement.ref_pt1 = Eigen::Vector3d(ref_pt1.x, ref_pt1.y, ref_pt1.z);

      auto& ref_pt2 =
          ref_->edges.strong.cloud.points.at(point_search_ind.at(1));
      measurement.ref_pt2 = Eigen::Vector3d(ref_pt2.x, ref_pt2.y, ref_pt2.z);

      edge_measurements_.push_back(measurement);
      continue;
    }

    // check if weak features exist, if not then skip feature
    if (ref_->edges.weak.cloud.empty()) { continue; }
    ref_->edges.weak.BuildKDTree(false);

    // search for correspondence in weak features
    point_search_ind.clear();
    point_search_sq_dist.clear();
    num_returned = ref_->edges.weak.kdtree->nearestKSearch(
        search_pt, 2, point_search_ind, point_search_sq_dist);

    if (num_returned != 2) { continue; }

    // if both correspondences are close enough, add measurement, else skip
    // feature
    if (point_search_sq_dist.at(0) < params_->max_correspondence_distance &&
        point_search_sq_dist.at(1) < params_->max_correspondence_distance) {
      EdgeMeasurement measurement;
      measurement.query_pt =
          Eigen::Vector3d(query_pt.x, query_pt.y, query_pt.z);

      auto& ref_pt1 = ref_->edges.weak.cloud.points.at(point_search_ind.at(0));
      measurement.ref_pt1 = Eigen::Vector3d(ref_pt1.x, ref_pt1.y, ref_pt1.z);

      auto& ref_pt2 = ref_->edges.weak.cloud.points.at(point_search_ind.at(1));
      measurement.ref_pt2 = Eigen::Vector3d(ref_pt2.x, ref_pt2.y, ref_pt2.z);

      edge_measurements_.push_back(measurement);
    }
  }
  return true;
}

bool LoamScanRegistration::GetSurfaceMeasurements() {
  surface_measurements_.clear();

  // transform target cloud to reference frame with current estimate
  PointCloudIRT tgt_features;
  if (T_REF_TGT_.isIdentity()) {
    tgt_features = tgt_->surfaces.strong.cloud;
  } else {
    pcl::transformPointCloud(tgt_->surfaces.strong.cloud, tgt_features,
                             T_REF_TGT_);
  }

  // build kdtrees if not already done
  if (ref_->surfaces.strong.cloud.size() > 0) {
    ref_->surfaces.strong.BuildKDTree(false);
  } else {
    BEAM_ERROR(
        "Reference cloud has no surface features, aborting registration.");
    return false;
  }

  for (size_t tgt_iter = 0; tgt_iter < tgt_features.size(); tgt_iter++) {
    auto search_pt = tgt_features.points.at(tgt_iter);
    auto& query_pt = tgt_->surfaces.strong.cloud.at(tgt_iter);

    // search for correspondence in strong fetures
    std::vector<uint32_t> point_search_ind;
    std::vector<float> point_search_sq_dist;
    size_t num_returned = ref_->surfaces.strong.kdtree->nearestKSearch(
        search_pt, 3, point_search_ind, point_search_sq_dist);

    // if both correspondences are close enough, add measurement, else check in
    // weak features
    if (num_returned != 3) {
      // do nothing, we will check weak features
    } else if (point_search_sq_dist.at(0) <
                   params_->max_correspondence_distance &&
               point_search_sq_dist.at(1) <
                   params_->max_correspondence_distance) {
      SurfaceMeasurement measurement;
      measurement.query_pt =
          Eigen::Vector3d(query_pt.x, query_pt.y, query_pt.z);

      auto& ref_pt1 =
          ref_->surfaces.strong.cloud.points.at(point_search_ind.at(0));
      measurement.ref_pt1 = Eigen::Vector3d(ref_pt1.x, ref_pt1.y, ref_pt1.z);

      auto& ref_pt2 =
          ref_->surfaces.strong.cloud.points.at(point_search_ind.at(1));
      measurement.ref_pt2 = Eigen::Vector3d(ref_pt2.x, ref_pt2.y, ref_pt2.z);

      auto& ref_pt3 =
          ref_->surfaces.strong.cloud.points.at(point_search_ind.at(2));
      measurement.ref_pt3 = Eigen::Vector3d(ref_pt3.x, ref_pt3.y, ref_pt3.z);

      surface_measurements_.push_back(measurement);
      continue;
    }

    // check weak features exist, if not then skip feature
    if (ref_->surfaces.weak.cloud.empty()) { continue; }
    ref_->surfaces.weak.BuildKDTree(false);

    // search for correspondence in weak fetures
    point_search_ind.clear();
    point_search_sq_dist.clear();
    num_returned = ref_->surfaces.weak.kdtree->nearestKSearch(
        search_pt, 3, point_search_ind, point_search_sq_dist);

    if (num_returned != 3) { continue; }

    // if both correspondences are close enough, add measurement, else skip
    // feature
    if (point_search_sq_dist.at(0) < params_->max_correspondence_distance &&
        point_search_sq_dist.at(1) < params_->max_correspondence_distance) {
      SurfaceMeasurement measurement;

      measurement.query_pt =
          Eigen::Vector3d(query_pt.x, query_pt.y, query_pt.z);

      auto& ref_pt1 =
          ref_->surfaces.weak.cloud.points.at(point_search_ind.at(0));
      measurement.ref_pt1 = Eigen::Vector3d(ref_pt1.x, ref_pt1.y, ref_pt1.z);

      auto& ref_pt2 =
          ref_->surfaces.weak.cloud.points.at(point_search_ind.at(1));
      measurement.ref_pt2 = Eigen::Vector3d(ref_pt2.x, ref_pt2.y, ref_pt2.z);

      auto& ref_pt3 =
          ref_->surfaces.weak.cloud.points.at(point_search_ind.at(2));
      measurement.ref_pt3 = Eigen::Vector3d(ref_pt3.x, ref_pt3.y, ref_pt3.z);

      surface_measurements_.push_back(measurement);
    }
  }
  return true;
}

static void TestCost(const Eigen::Vector3d& _P_TGT,
                     const Eigen::Vector3d& _P_REF1,
                     const Eigen::Vector3d& _P_REF2,
                     const std::vector<double>& _qt_REF_TGT) {
  double P_TGT[3];
  P_TGT[0] = _P_TGT[0];
  P_TGT[1] = _P_TGT[1];
  P_TGT[2] = _P_TGT[2];
  double P_REF[3];
  double qt_REF_TGT[7];
  qt_REF_TGT[0] = _qt_REF_TGT[0];
  qt_REF_TGT[1] = _qt_REF_TGT[1];
  qt_REF_TGT[2] = _qt_REF_TGT[2];
  qt_REF_TGT[3] = _qt_REF_TGT[3];
  qt_REF_TGT[4] = _qt_REF_TGT[4];
  qt_REF_TGT[5] = _qt_REF_TGT[5];
  qt_REF_TGT[6] = _qt_REF_TGT[6];
  ceres::QuaternionRotatePoint(qt_REF_TGT, P_TGT, P_REF);
  P_REF[0] += qt_REF_TGT[4];
  P_REF[1] += qt_REF_TGT[5];
  P_REF[2] += qt_REF_TGT[6];

  double d1[3], d2[3], d12[3];
  d1[0] = P_REF[0] - _P_REF1[0];
  d1[1] = P_REF[1] - _P_REF1[1];
  d1[2] = P_REF[2] - _P_REF1[2];

  d2[0] = P_REF[0] - _P_REF2[0];
  d2[1] = P_REF[1] - _P_REF2[1];
  d2[2] = P_REF[2] - _P_REF2[2];

  d12[0] = _P_REF1[0] - _P_REF2[0];
  d12[1] = _P_REF1[1] - _P_REF2[1];
  d12[2] = _P_REF1[2] - _P_REF2[2];

  double cross[3];
  ceres::CrossProduct(d1, d2, cross);

  double norm =
      sqrt(cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]);
  double norm12 = sqrt(d12[0] * d12[0] + d12[1] * d12[1] + d12[2] * d12[2]);

  double residual = norm / norm12;

  std::cout << "norm12: " << norm12 << "\n";
  std::cout << "residual: " << residual << "\n";
}

bool LoamScanRegistration::Solve(int iteration) {
  if (params_->min_number_measurements >
      edge_measurements_.size() + surface_measurements_.size()) {
    BEAM_ERROR(
        "Insufficient number of measurements for scan registration, aborting.");
    return false;
  }

  // setup problem
  ceres::Solver::Options solver_options =
      params_->optimizer_params.SolverOptions();

  ceres::Problem::Options problem_options =
      params_->optimizer_params.ProblemOptions();

  std::shared_ptr<ceres::Problem> problem =
      std::make_shared<ceres::Problem>(problem_options);

  std::unique_ptr<ceres::LossFunction> loss_function =
      params_->optimizer_params.LossFunction();

  std::unique_ptr<ceres::LocalParameterization> parameterization =
      params_->optimizer_params.SE3QuatTransLocalParametrization();

  // [Hack] in cases where the lidar is stationary, sometimes the residuals
  // become zero and the optimizer will fail. This adds a small perturbation to
  // the transform for the case where the transform is identity
  if (T_REF_TGT_.isIdentity()) {
    T_REF_TGT_(0, 3) = T_REF_TGT_(0, 3) + 0.005;
    T_REF_TGT_(1, 3) = T_REF_TGT_(1, 3) + 0.005;
    T_REF_TGT_(2, 3) = T_REF_TGT_(2, 3) + 0.005;
  }

  // add pose parameters
  Eigen::Matrix3d R = T_REF_TGT_.block(0, 0, 3, 3);
  Eigen::Quaternion<double> q = Eigen::Quaternion<double>(R);
  std::vector<double> pose{
      q.w(),           q.x(), q.y(), q.z(), T_REF_TGT_(0, 3), T_REF_TGT_(1, 3),
      T_REF_TGT_(2, 3)};
  problem->AddParameterBlock(&(pose[0]), 7, parameterization.get());

  // add edge measuremnts
  for (EdgeMeasurement m : edge_measurements_) {
    std::unique_ptr<ceres::CostFunction> cost_function(
        CeresPointToLineCostFunction::Create(m.query_pt, m.ref_pt1, m.ref_pt2));
    problem->AddResidualBlock(cost_function.release(), loss_function.get(),
                              &(pose[0]));
  }

  // add planar measuremnts
  for (SurfaceMeasurement m : surface_measurements_) {
    std::unique_ptr<ceres::CostFunction> cost_function(
        CeresPointToPlaneCostFunction::Create(m.query_pt, m.ref_pt1, m.ref_pt2,
                                              m.ref_pt3));
    problem->AddResidualBlock(cost_function.release(), loss_function.get(),
                              &(pose[0]));
  }

  // solve problem
  ceres::Solver::Summary ceres_summary;
  ceres::Solve(solver_options, problem.get(), &ceres_summary);

  if (params_->output_ceres_summary) {
    BEAM_INFO("Outputting ceres summary for iteration {}", iteration);
    std::string report = ceres_summary.FullReport();
    std::cout << report << "\n";
  }

  // update current pose
  T_REF_TGT_ = beam::QuaternionAndTranslationToTransformMatrix(pose);

  if (params_->output_optimization_summary) {
    // add ceres results to summary
    optimization_summary_.ceres_termination = ceres_summary.message;
    optimization_summary_.ceres_residuals = ceres_summary.num_residuals;
    optimization_summary_.ceres_iterations = ceres_summary.iterations.size();
    optimization_summary_.ceres_initial_cost = ceres_summary.initial_cost;
    optimization_summary_.ceres_final_cost = ceres_summary.final_cost;

    // add measurement results to summary
    optimization_summary_.correspondence_iteration_number = iteration;
    optimization_summary_.surface_measurements = surface_measurements_.size();
    optimization_summary_.edge_measurements = edge_measurements_.size();
  }

  // compute covariance
  ceres::Covariance::Options cov_options;
  ceres::Covariance covariance(cov_options);
  std::vector<const double*> covariance_block;
  covariance_block.push_back(&(pose[0]));
  covariance.Compute(covariance_block, problem.get());

  // setup covariance to return as eigen matrix
  double covariance_arr[7 * 7];
  covariance.GetCovarianceBlock(&(pose[0]), &(pose[0]), covariance_arr);
  covariance_ = Eigen::Matrix<double, 7, 7>(covariance_arr);
  return ceres_summary.IsSolutionUsable();
}

bool LoamScanRegistration::HasConverged(int iteration) {
  if (!params_->iterate_correspondences) {
    if (params_->output_optimization_summary) {
      optimization_summary_.translation_change_m = 0;
      optimization_summary_.rotation_change_deg = 0;
      optimization_summary_.correspondence_termination =
          "TERMINATE: Iterate correspondences set to false.";
    }
    return true;
  } else if (iteration == 0) {
    if (params_->output_optimization_summary) {
      optimization_summary_.translation_change_m = 0;
      optimization_summary_.rotation_change_deg = 0;
      optimization_summary_.correspondence_termination =
          "CONTINUE: at first iteration.";
    }
    return false;
  } else if (iteration >= params_->max_correspondence_iterations) {
    if (params_->output_optimization_summary) {
      optimization_summary_.translation_change_m = 0;
      optimization_summary_.rotation_change_deg = 0;
      optimization_summary_.correspondence_termination =
          "TERMINATE: reached max correspondence iterations.";
    }
    return true;
  }

  Eigen::Matrix4d T_diff =
      beam::InvertTransform(T_REF_TGT_prev_iter_) * T_REF_TGT_;

  double t_diff_norm = T_diff.block(0, 3, 3, 1).norm();
  if (t_diff_norm > params_->convergence_criteria_translation_m) {
    if (params_->output_optimization_summary) {
      optimization_summary_.translation_change_m = t_diff_norm;
      optimization_summary_.rotation_change_deg = 0;
      optimization_summary_.correspondence_termination =
          "CONTINUE: change in translation larger than citeria (" +
          std::to_string(params_->convergence_criteria_translation_m) + ").";
    }
    return false;
  }

  Eigen::Matrix3d R_diff = T_diff.block(0, 0, 3, 3);
  double angle_rad = Eigen::AngleAxis<double>(R_diff).angle();
  if (std::abs(angle_rad) >
      beam::Deg2Rad(params_->convergence_criteria_rotation_deg)) {
    if (params_->output_optimization_summary) {
      optimization_summary_.translation_change_m = t_diff_norm;
      optimization_summary_.rotation_change_deg =
          beam::Rad2Deg(std::abs(angle_rad));
      optimization_summary_.correspondence_termination =
          "CONTINUE: change in rotation larger than citeria (" +
          std::to_string(params_->convergence_criteria_rotation_deg) + ").";
    }
    return false;
  }

  if (params_->output_optimization_summary) {
    optimization_summary_.translation_change_m = t_diff_norm;
    optimization_summary_.rotation_change_deg =
        beam::Rad2Deg(std::abs(angle_rad));
    optimization_summary_.correspondence_termination =
        "TERMINATE: change in rotation is less than criteria (" +
        std::to_string(params_->convergence_criteria_rotation_deg) +
        ") and change in translation is less than criteria (" +
        std::to_string(params_->convergence_criteria_translation_m) + ")";
  }

  return true;
}

void LoamScanRegistration::OutputResults(int iteration) {
  if (params_->output_optimization_summary) { optimization_summary_.Print(); }

  // output scan registration results
  if (!output_results_) { return; }

  if (!boost::filesystem::exists(debug_output_path_)) {
    BEAM_ERROR("Debug output path does not exist, not saving results.");
    return;
  }

  std::string current_dir = beam::CombinePaths(
      {debug_output_path_stamped_, "iteration" + std::to_string(iteration)});
  boost::filesystem::create_directory(current_dir);
  ref_->SaveCombined(current_dir, "referece_cloud.pcd");

  LoamPointCloud target_aligned = tgt_->Copy();
  target_aligned.TransformPointCloud(T_REF_TGT_);
  target_aligned.SaveCombined(current_dir, "target_aligned.pcd");

  LoamPointCloud target_initial = tgt_->Copy();
  target_initial.TransformPointCloud(T_REF_TGT_prev_iter_);
  target_initial.SaveCombined(current_dir, "target_initial.pcd");
}

void LoamScanRegistration::OptimizationSummary::Print() {
  std::cout << "-------- OPTIMIZATION SUMMARY --------\n"
            << "correspondence_iteration_number: "
            << correspondence_iteration_number << "\n"
            << "surface_measurements: " << surface_measurements << "\n"
            << "edge_measurements: " << edge_measurements << "\n"
            << "translation_change (meters): " << translation_change_m << "\n"
            << "rotation_change (degrees): " << rotation_change_deg << "\n"
            << "ceres_termination: " << ceres_termination << "\n"
            << "ceres_initial_cost: " << ceres_initial_cost << "\n"
            << "ceres_final_cost: " << ceres_final_cost << "\n"
            << "correspondence_termination: " << correspondence_termination
            << "\n";
}

void LoamScanRegistration::SaveResults(const std::string& output_path,
                                       const std::string& prefix) const {
  if (!boost::filesystem::exists(output_path)) {
    BEAM_WARN(
        "Output path does not exist, cannot save matcher results. Input: {}",
        output_path);
    return;
  }

  std::string output_final = beam::CombinePaths(output_path, prefix);
  BEAM_INFO("saving registration results to: {}", output_final + "*");

  ref_->SaveCombined(output_path, prefix + "_referece_cloud.pcd", 0, 0, 255);
  tgt_->SaveCombined(output_path, prefix + "_target_initial.pcd", 255, 0, 0);

  LoamPointCloud target_aligned = tgt_->Copy();
  target_aligned.TransformPointCloud(T_REF_TGT_);
  target_aligned.SaveCombined(output_path, prefix + "_target_aligned.pcd", 0,
                              255, 0);
}

Eigen::Matrix<double, 7, 7> LoamScanRegistration::GetCovariance() const {
  return covariance_;
}

} // namespace beam_matching
