#include <beam_matching/LoamMatcher.h>

#include <fstream>
#include <iostream>

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include <beam_utils/log.h>

namespace beam_matching {

LoamMatcher::LoamMatcher() {
  params_ = std::make_shared<LoamParams>();
  loam_scan_registration_ = std::make_unique<LoamScanRegistration>(params_);
}

LoamMatcher::LoamMatcher(const LoamParams& params) {
  params_ = std::make_shared<LoamParams>();
  *params_ = params;
  loam_scan_registration_ = std::make_unique<LoamScanRegistration>(params_);
}

LoamMatcher::~LoamMatcher() {
  if (loam_scan_registration_) { loam_scan_registration_.reset(); }
  if (params_) { params_.reset(); }
  if (ref_) { ref_.reset(); }
  if (target_) { target_.reset(); }
}

void LoamMatcher::SetParams(const LoamParams& params) {
  params_ = std::make_shared<LoamParams>(params);
  loam_scan_registration_ = std::make_unique<LoamScanRegistration>(params_);
}

void LoamMatcher::SetRef(const LoamPointCloudPtr& ref) {
  this->ref_ = ref;
}

void LoamMatcher::SetTarget(const LoamPointCloudPtr& target) {
  this->target_ = target;
}

bool LoamMatcher::Match() {
  bool registration_successful =
      loam_scan_registration_->RegisterScans(ref_, target_);
  const Eigen::Matrix4d& T_REF_TGT = loam_scan_registration_->GetT_REF_TGT();
  result_ = Eigen::Affine3d(beam::InvertTransform(T_REF_TGT));
  return registration_successful;
}

} // namespace beam_matching
