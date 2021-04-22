#include <beam_matching/LoamMatcher.h>

#include <fstream>
#include <iostream>

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include <beam_utils/log.h>

namespace beam_matching {

LoamMatcherParams::LoamMatcherParams(std::string& param_config) {
  BEAM_INFO("Loading LOAM matcher config file: {}", param_config.c_str());

  nlohmann::json J;
  std::ifstream file(param_config);
  file >> J;
}

LoamMatcher::LoamMatcher(LoamMatcherParams params) : params_(params) {
  SetLoamParams();
}

LoamMatcher::~LoamMatcher() {}

void LoamMatcher::SetParams(LoamMatcherParams params) {
  this->params_ = params;
  SetLoamParams();
}

void LoamMatcher::SetRef(const LoamPointCloudPtr& ref) {
  this->ref_ = ref;
}

void LoamMatcher::SetTarget(const LoamPointCloudPtr& target) {
  this->target_ = target;
}

bool LoamMatcher::Match() {
  // TODO:
  // registration_.
  return true;
}

} // namespace beam_matching
