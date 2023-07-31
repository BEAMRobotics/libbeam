#include <beam_matching/LoamMatcher.h>

#include <fstream>
#include <iostream>

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include <beam_utils/log.h>
#include <beam_utils/math.h>
#include <beam_utils/se3.h>

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

void LoamMatcher::SaveResults(const std::string& output_dir,
                              const std::string& prefix) {
  loam_scan_registration_->SaveResults(output_dir, prefix);
}

void LoamMatcher::CalculateCovariance() {
  Eigen::Matrix<double, 7, 7> covariance_full =
      loam_scan_registration_->GetCovariance();
  // loam scan registration returns a covariance of form:
  // [dqw, dqx, dqy, dqz, dx, dy, dz]
  // whereas Matcher base class requires the form: [dx, dy, dz, dqx, dqy, dqz]
  // therefore we need to drop the first column and row and then convert
  // according to:
  // | A B | ---\  | D C | 
  // | C D | ---/  | B A |
  covariance_.block(0,0,3,3) = covariance_full.block(4,4,3,3); 
  covariance_.block(0,3,3,3) = covariance_full.block(4,1,3,3); 
  covariance_.block(3,0,3,3) = covariance_full.block(1,4,3,3); 
  covariance_.block(3,3,3,3) = covariance_full.block(1,1,3,3); 
}

} // namespace beam_matching
