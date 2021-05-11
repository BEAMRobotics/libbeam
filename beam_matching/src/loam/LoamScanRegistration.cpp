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
    OutputResults(iteration);
    if (HasConverged(iteration)) { break; }
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
  PointCloud tgt_features;
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
  if (ref_->edges.weak.cloud.size() > 0) {
    ref_->edges.weak.BuildKDTree(false);
  }

  for (size_t tgt_iter = 0; tgt_iter < tgt_features.size(); tgt_iter++) {
    auto& search_pt = tgt_features.points[tgt_iter];
    auto& query_pt = tgt_->edges.strong.cloud[tgt_iter];

    // search for correspondence in strong fetures
    std::vector<int> point_search_ind;
    std::vector<float> point_search_sq_dist;
    ref_->edges.strong.kdtree.nearestKSearch(search_pt, 2, point_search_ind,
                                             point_search_sq_dist);

    // if both correspondences are close enough, add measurement, else check in
    // weak features
    if (point_search_sq_dist[0] < params_->max_correspondence_distance &&
        point_search_sq_dist[1] < params_->max_correspondence_distance) {
      EdgeMeasurement measurement;

      measurement.query_pt =
          Eigen::Vector3d(query_pt.x, query_pt.y, query_pt.z);

      auto& ref_pt1 = ref_->edges.strong.cloud.points[point_search_ind[0]];
      measurement.ref_pt1 = Eigen::Vector3d(ref_pt1.x, ref_pt1.y, ref_pt1.z);

      auto& ref_pt2 = ref_->edges.strong.cloud.points[point_search_ind[1]];
      measurement.ref_pt2 = Eigen::Vector3d(ref_pt2.x, ref_pt2.y, ref_pt2.z);

      edge_measurements_.push_back(measurement);
      continue;
    }

    // search for correspondence in strong fetures
    point_search_ind.clear();
    point_search_sq_dist.clear();
    ref_->edges.weak.kdtree.nearestKSearch(search_pt, 2, point_search_ind,
                                           point_search_sq_dist);

    // if both correspondences are close enough, add measurement, else skip
    // feature
    if (point_search_sq_dist[0] < params_->max_correspondence_distance &&
        point_search_sq_dist[1] < params_->max_correspondence_distance) {
      EdgeMeasurement measurement;
      measurement.query_pt =
          Eigen::Vector3d(query_pt.x, query_pt.y, query_pt.z);

      auto& ref_pt1 = ref_->edges.weak.cloud.points[point_search_ind[0]];
      measurement.ref_pt1 = Eigen::Vector3d(ref_pt1.x, ref_pt1.y, ref_pt1.z);

      auto& ref_pt2 = ref_->edges.weak.cloud.points[point_search_ind[1]];
      measurement.ref_pt2 = Eigen::Vector3d(ref_pt2.x, ref_pt2.y, ref_pt2.z);

      edge_measurements_.push_back(measurement);
    }
  }
  return true;
}

bool LoamScanRegistration::GetSurfaceMeasurements() {
  surface_measurements_.clear();

  // transform target cloud to reference frame with current estimate
  PointCloud tgt_features;
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
  if (ref_->surfaces.weak.cloud.size() > 0) {
    ref_->surfaces.weak.BuildKDTree(false);
  }

  for (size_t tgt_iter = 0; tgt_iter < tgt_features.size(); tgt_iter++) {
    auto search_pt = tgt_features.points[tgt_iter];
    auto& query_pt = tgt_->surfaces.strong.cloud[tgt_iter];

    // search for correspondence in strong fetures
    std::vector<int> point_search_ind;
    std::vector<float> point_search_sq_dist;
    ref_->surfaces.strong.kdtree.nearestKSearch(search_pt, 3, point_search_ind,
                                                point_search_sq_dist);

    // if both correspondences are close enough, add measurement, else check in
    // weak features
    if (point_search_sq_dist[0] < params_->max_correspondence_distance &&
        point_search_sq_dist[1] < params_->max_correspondence_distance) {
      SurfaceMeasurement measurement;
      measurement.query_pt =
          Eigen::Vector3d(query_pt.x, query_pt.y, query_pt.z);

      auto& ref_pt1 = ref_->surfaces.strong.cloud.points[point_search_ind[0]];
      measurement.ref_pt1 = Eigen::Vector3d(ref_pt1.x, ref_pt1.y, ref_pt1.z);

      auto& ref_pt2 = ref_->surfaces.strong.cloud.points[point_search_ind[1]];
      measurement.ref_pt2 = Eigen::Vector3d(ref_pt2.x, ref_pt2.y, ref_pt2.z);

      auto& ref_pt3 = ref_->surfaces.strong.cloud.points[point_search_ind[2]];
      measurement.ref_pt3 = Eigen::Vector3d(ref_pt3.x, ref_pt3.y, ref_pt3.z);

      surface_measurements_.push_back(measurement);
      continue;
    }

    // search for correspondence in strong fetures
    point_search_ind.clear();
    point_search_sq_dist.clear();
    ref_->surfaces.weak.kdtree.nearestKSearch(search_pt, 3, point_search_ind,
                                              point_search_sq_dist);

    // if both correspondences are close enough, add measurement, else skip
    // feature
    if (point_search_sq_dist[0] < params_->max_correspondence_distance &&
        point_search_sq_dist[1] < params_->max_correspondence_distance) {
      SurfaceMeasurement measurement;
      measurement.query_pt =
          Eigen::Vector3d(query_pt.x, query_pt.y, query_pt.z);

      auto& ref_pt1 = ref_->surfaces.weak.cloud.points[point_search_ind[0]];
      measurement.ref_pt1 = Eigen::Vector3d(ref_pt1.x, ref_pt1.y, ref_pt1.z);

      auto& ref_pt2 = ref_->surfaces.weak.cloud.points[point_search_ind[1]];
      measurement.ref_pt2 = Eigen::Vector3d(ref_pt2.x, ref_pt2.y, ref_pt2.z);

      auto& ref_pt3 = ref_->surfaces.strong.cloud.points[point_search_ind[2]];
      measurement.ref_pt3 = Eigen::Vector3d(ref_pt3.x, ref_pt3.y, ref_pt3.z);

      surface_measurements_.push_back(measurement);
    }
  }
  return true;
}

bool LoamScanRegistration::Solve(int iteration) {
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

  return ceres_summary.IsSolutionUsable();
}

bool LoamScanRegistration::HasConverged(int iteration) {
  if (!params_->iterate_correspondences) { return true; }

  if (iteration == 0) { return false; }

  if (iteration >= params_->max_correspondence_iterations) { return true; }

  Eigen::Matrix4d T_diff =
      beam::InvertTransform(T_REF_TGT_prev_iter_) * T_REF_TGT_;

  Eigen::Vector3d t_diff = T_diff.block(0, 3, 3, 1);
  if (t_diff.norm() > params_->convergence_criteria_translation_m) {
    return false;
  }

  Eigen::Matrix3d R_diff = T_diff.block(0, 0, 3, 3);
  double angle = Eigen::AngleAxis<double>(R_diff).angle();
  if (std::abs(angle) >
      beam::Deg2Rad(params_->convergence_criteria_rotation_deg)) {
    return false;
  }

  return true;
}

void LoamScanRegistration::OutputResults(int iteration) {
  if(!output_results_){return;}
  
  if (!boost::filesystem::exists(debug_output_path_)) {
    BEAM_ERROR("Debug output path does not exist, not saving results.");
    return;
  }

  std::string current_dir =
      debug_output_path_ + "iteration" + std::to_string(iteration) + "/";
  boost::filesystem::create_directory(current_dir);

  boost::filesystem::create_directory(current_dir + "referece_cloud/");
  boost::filesystem::create_directory(current_dir + "target_aligned/");
  boost::filesystem::create_directory(current_dir + "target_initial/");
  ref_->Save(current_dir + "referece_cloud/", true);

  LoamPointCloud target_aligned = *tgt_;
  target_aligned.TransformPointCloud(T_REF_TGT_);
  target_aligned.Save(current_dir + "target_aligned/", true);

  LoamPointCloud target_initial = *tgt_;
  target_initial.TransformPointCloud(T_REF_TGT_prev_iter_);
  target_initial.Save(current_dir + "target_initial/", true);
}

} // namespace beam_matching
