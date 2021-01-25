#include "beam_matching/GicpMatcher.hpp"

#include <beam_utils/log.h>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace beam_matching {

GicpMatcherParams::GicpMatcherParams(const std::string& config_path) {
  BEAM_INFO("Loading file: {}", config_path.c_str());

  json J;
  std::ifstream file(config_path);
  file >> J;

  this->corr_rand = J["corr_rand"];
  this->max_iter = J["max_iter"];
  this->r_eps = J["r_eps"];
  this->fit_eps = J["fit_eps"];
  this->res = J["res"];
}

GicpMatcher::GicpMatcher(const GicpMatcherParams params) : params_(params) {
  SetGicpParams();
}

void GicpMatcher::SetParams(const GicpMatcherParams params) {
  this->params_ = params;
  SetGicpParams();
}

void GicpMatcher::SetGicpParams() {
  this->ref_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
  this->target_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
  this->final_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();

  if (params_.res > 0) {
    this->resolution_ = params_.res;
    this->filter_.setLeafSize(params_.res, params_.res, params_.res);
  } else {
    this->resolution_ = -1;
  }
  this->gicp_.setCorrespondenceRandomness(this->params_.corr_rand);
  this->gicp_.setMaximumIterations(this->params_.max_iter);
  this->gicp_.setRotationEpsilon(this->params_.r_eps);
  this->gicp_.setEuclideanFitnessEpsilon(this->params_.fit_eps);
}

void GicpMatcher::SetRef(const PCLPointCloudPtr& ref) {
  if (this->resolution_ > 0) {
    this->filter_.setInputCloud(ref);
    this->filter_.filter(*(this->ref_));
  } else {
    this->ref_ = ref;
  }
  this->gicp_.setInputSource(this->ref_);
}

void GicpMatcher::SetTarget(const PCLPointCloudPtr& target) {
  if (resolution_ > 0) {
    this->filter_.setInputCloud(target);
    this->filter_.filter(*(this->target_));
  } else {
    this->target_ = target;
  }
  this->gicp_.setInputTarget(this->target_);
}

bool GicpMatcher::Match() {
  this->gicp_.align(*(this->final_));
  if (this->gicp_.hasConverged()) {
    this->result_.matrix() = gicp_.getFinalTransformation().cast<double>();
    return true;
  }
  return false;
}

} // namespace beam_matching
