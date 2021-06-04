#include <beam_matching/GicpMatcher.h>

#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

#include <beam_utils/log.h>
#include <beam_utils/filesystem.h>

namespace beam_matching {

GicpMatcher::Params::Params(const std::string& config_path) {
  std::string read_file = config_path;
  if (config_path.empty()) {
    return;
  } else if (!boost::filesystem::exists(config_path)) {
    BEAM_WARN("Invalid matcher config path, file does not exist, using "
              "default. Input: {}",
              config_path);
    return;
  } else if (config_path == "DEFAULT_PATH") {
    std::string default_path = beam::LibbeamRoot();
    default_path += "beam_matching/config/gicp.json";
    if (!boost::filesystem::exists(default_path)) {
      BEAM_WARN("Could not find default gicp config at: {}. Using "
                "default params.",
                default_path);
      return;
    }
    read_file = default_path;
  }

  BEAM_INFO("Loading GICP matcher file: {}", read_file);

  nlohmann::json J;
  std::ifstream file(read_file);
  file >> J;

  this->corr_rand = J["corr_rand"];
  this->max_iter = J["max_iter"];
  this->r_eps = J["r_eps"];
  this->fit_eps = J["fit_eps"];
  this->res = J["res"];
}

GicpMatcher::GicpMatcher(const Params params) : params_(params) {
  SetGicpParams();
}

void GicpMatcher::SetParams(const Params params) {
  this->params_ = params;
  SetGicpParams();
}

void GicpMatcher::SetGicpParams() {
  this->ref_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
  this->target_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
  this->final_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();

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

void GicpMatcher::SetRef(const PointCloudPtr& ref) {
  if (this->resolution_ > 0) {
    this->filter_.setInputCloud(ref);
    this->filter_.filter(*(this->ref_));
  } else {
    this->ref_ = ref;
  }
  this->gicp_.setInputSource(this->ref_);
}

void GicpMatcher::SetTarget(const PointCloudPtr& target) {
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
