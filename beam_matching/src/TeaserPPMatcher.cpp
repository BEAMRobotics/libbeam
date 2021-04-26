#include <beam_matching/TeaserPPMatcher.h>

#include <fstream>
#include <iostream>
#include <sys/sysinfo.h>

#include <nlohmann/json.hpp>
#include <pcl/common/transforms.h>

#include <beam_utils/log.h>

using json = nlohmann::json;

namespace beam_matching {

TeaserPPMatcherParams::TeaserPPMatcherParams(const std::string& config_path) {
  BEAM_INFO("Loading Teaser++ matcher file: {}", config_path.c_str());

  json J;
  std::ifstream file(config_path);
  file >> J;

  noise_bound = J["noise_bound"];
  cbar2 = J["cbar2"];
  estimate_scaling = J["estimate_scaling"];
  rotation_max_iterations = J["rotation_max_iterations"];
  rotation_gnc_factor = J["rotation_gnc_factor"];
  std::string rot_algo = J["rotation_estimation_algorithm"];
  if (rot_algo == "GNC_TLS") {
    rotation_estimation_algorithm = RotAlgo::GNC_TLS;
  } else if (rot_algo == "FGR") {
    rotation_estimation_algorithm = RotAlgo::FGR;
  } else {
    BEAM_ERROR(
        "Invalid rotation_estimation_algorithm param. Options: GNC_TLS, FGR");
    throw std::invalid_argument{"Invalid rotation_estimation_algorithm param."};
  }
  rotation_cost_threshold = J["rotation_cost_threshold"];
  res = J["res"];
}

teaser::RobustRegistrationSolver::Params
    TeaserPPMatcherParams::GetSolverParams() {
  teaser::RobustRegistrationSolver::Params params;
  params.noise_bound = noise_bound;
  params.cbar2 = cbar2;
  params.estimate_scaling = estimate_scaling;
  params.rotation_max_iterations = rotation_max_iterations;
  params.rotation_gnc_factor = rotation_gnc_factor;
  params.rotation_estimation_algorithm = rotation_estimation_algorithm;
  params.rotation_cost_threshold = rotation_cost_threshold;
  return params;
}

TeaserPPMatcher::TeaserPPMatcher(const TeaserPPMatcherParams params)
    : params_(params) {
  SetTeaserPPParams();
}

void TeaserPPMatcher::SetParams(const TeaserPPMatcherParams params) {
  params_ = params;
  SetTeaserPPParams();
}

void TeaserPPMatcher::SetTeaserPPParams() {
  ref_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
  target_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();

  if (params_.res > 0) {
    resolution_ = params_.res;
    filter_.setLeafSize(params_.res, params_.res, params_.res);
  } else {
    resolution_ = -1;
  }

  auto solver_params = params_.GetSolverParams();
  teaserpp_ = teaser::RobustRegistrationSolver(solver_params);
}

void TeaserPPMatcher::SetRef(const PointCloudPtr& ref) {
  if (resolution_ > 0) {
    filter_.setInputCloud(ref);
    filter_.filter(*ref_);
  } else {
    ref_ = ref;
  }
}

void TeaserPPMatcher::SetTarget(const PointCloudPtr& target) {
  if (resolution_ > 0) {
    filter_.setInputCloud(target);
    filter_.filter(*target_);
  } else {
    target_ = target;
  }
}

bool TeaserPPMatcher::Match() {
  if (ref_->size() == 0 || target_->size() == 0) { return false; }

  // Convert the point clouds to Eigen
  Eigen::Matrix<double, 3, Eigen::Dynamic> src_cloud(3, ref_->size());
  for (size_t i = 0; i < ref_->size(); ++i) {
    src_cloud.col(i) << ref_->points.at(i).x, ref_->points.at(i).y,
        ref_->points.at(i).z;
  }
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt_cloud(3, target_->size());
  for (size_t i = 0; i < target_->size(); ++i) {
    tgt_cloud.col(i) << target_->points.at(i).x, target_->points.at(i).y,
        target_->points.at(i).z;
  }

  // check that clouds aren't too large for the current memory
  double max_size = static_cast<double>(ref_->size());
  if (target_->size() > ref_->size()) {
    max_size = static_cast<double>(target_->size());
  }
  // see: teaser::RobustRegistrationSolver::computeTIMs
  double num_GBs_required =
      (max_size / 1000 * (max_size / 1000 - 1) / 2) * 3 / 1000 * 8;
  struct sysinfo info;
  sysinfo(&info);
  auto free_mem_bytes = info.freeram * info.mem_unit;
  double num_GBs_available = static_cast<double>(free_mem_bytes) / 1000000000;
  if (num_GBs_available < num_GBs_required) {
    BEAM_WARN("Available RAM may not be sufficient for Teaser++ with point "
              "cloud of size {}. Available: {}GB, required: {}GB",
              max_size, num_GBs_available, num_GBs_required);
  }
  
  // solve, get solution & return validity
  teaserpp_.solve(src_cloud, tgt_cloud);
  teaser::RegistrationSolution solution = teaserpp_.getSolution();
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
  result.block(0, 3, 3, 1) = solution.translation;
  result.block(0, 0, 3, 3) = solution.rotation;
  result_ = Eigen::Affine3d(result);
  return solution.valid;
}

PointCloudPtr TeaserPPMatcher::GetAlignedRefCloud() {
  PointCloudPtr final = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
  pcl::transformPointCloud(*ref_, *final, result_);
  return final;
}

} // namespace beam_matching
