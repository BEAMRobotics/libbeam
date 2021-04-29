#include <beam_matching/LoamMatcher.h>

#include <fstream>
#include <iostream>

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include <beam_utils/log.h>

namespace beam_matching {

LoamMatcher::LoamMatcher() {
  params_ = std::make_shared<LoamParams>();
  feature_extractor_ = std::make_unique<LoamFeatureExtractor>(params_);
  loam_scan_registration_ = std::make_unique<LoamScanRegistration>(params_);
}

LoamMatcher::LoamMatcher(const LoamParams& params) {
  params_ = std::make_shared<LoamParams>(params);
  feature_extractor_ = std::make_unique<LoamFeatureExtractor>(params_);
  loam_scan_registration_ = std::make_unique<LoamScanRegistration>(params_);
}

LoamMatcher::~LoamMatcher() {}

void LoamMatcher::SetParams(const LoamParams& params) {
  params_ = std::make_shared<LoamParams>(params);
  feature_extractor_ = std::make_unique<LoamFeatureExtractor>(params_);
  loam_scan_registration_ = std::make_unique<LoamScanRegistration>(params_);
}

void LoamMatcher::SetRef(const PointCloudPtr& ref) {
  LoamPointCloud ref_loam = feature_extractor_->ExtractFeatures(*ref);
  this->ref_ = std::make_shared<LoamPointCloud>(ref_loam);
}

void LoamMatcher::SetTarget(const PointCloudPtr& target) {
  LoamPointCloud target_loam = feature_extractor_->ExtractFeatures(*target);
  this->target_ = std::make_shared<LoamPointCloud>(target_loam);
}

bool LoamMatcher::Match() {
  bool registration_successful =
      loam_scan_registration_->RegisterScans(ref_, target_);
  result_ = Eigen::Affine3d(loam_scan_registration_->GetT_REF_TGT());
  return registration_successful;
}

} // namespace beam_matching
