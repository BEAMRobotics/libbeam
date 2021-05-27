#include <beam_matching/NdtMatcher.h>

#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

#include <beam_utils/utils.h>
#include <beam_utils/log.h>

namespace beam_matching {

NdtMatcher::Params::Params(const std::string &config_path) {
  BEAM_INFO("Loading NDT matcher config file: {}", config_path.c_str());

  nlohmann::json J;
  std::ifstream file(config_path);
  file >> J;

  this->step_size = J["step_size"];
  this->max_iter = J["max_iter"];
  this->t_eps = J["t_eps"];
  this->res = J["res"];
}

NdtMatcher::NdtMatcher(Params params) : params_(params) {
  SetNdtParams();
}

NdtMatcher::~NdtMatcher() {
    if (this->ref_) {
        this->ref_.reset();
    }
    if (this->target_) {
        this->target_.reset();
    }
    if (this->final_) {
        this->final_.reset();
    }
}

void NdtMatcher::SetParams(Params params) {
  this->params_ = params;
  SetNdtParams();
}

void NdtMatcher::SetNdtParams() {
  this->ref_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  this->target_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  this->final_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  if (this->params_.res < this->params_.min_res) {
      LOG_ERROR("Invalid resolution given, using minimum");
      this->params_.res = this->params_.min_res;
  }

  this->resolution_ = this->params_.res;

  this->ndt_.setTransformationEpsilon(this->params_.t_eps);
  this->ndt_.setStepSize(this->params_.step_size);
  this->ndt_.setResolution(this->params_.res);
  this->ndt_.setMaximumIterations(this->params_.max_iter);
}

void NdtMatcher::SetRef(const PointCloudPtr &ref) {
    this->ref_ = ref;
    this->ndt_.setInputSource(this->ref_);
}

void NdtMatcher::SetTarget(const PointCloudPtr &target) {
    this->target_ = target;
    this->ndt_.setInputTarget(this->target_);
}

bool NdtMatcher::Match() {
    this->ndt_.align(*(this->final_));
    if (this->ndt_.hasConverged()) {
        this->result_.matrix() = ndt_.getFinalTransformation().cast<double>();
        return true;
    }
    return false;
}

}  // namespace beam_matching
