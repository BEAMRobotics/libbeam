#include "beam_calibration/DistortionModel.h"
#include "beam_calibration/EquidistantDistortion.h"
#include "beam_calibration/RadTanDistortion.h"

namespace beam_calibration {

DistortionModel::DistortionModel(beam::VecX coeffs,
                                 beam_calibration::DistortionType type) {
  this->SetType(type);
  this->SetCoefficients(coeffs);
}

std::unique_ptr<DistortionModel> DistortionModel::Create(DistortionType type,
                                                         beam::VecX coeffs) {
  if (type == beam_calibration::DistortionType::RADTAN) {
    return std::unique_ptr<beam_calibration::RadTanDistortion>(
        new RadTanDistortion(coeffs, type));
  } else if (type == beam_calibration::DistortionType::NONE) {
    beam::VecX zeros = beam::VecX::Zero(5);
    return std::unique_ptr<beam_calibration::RadTanDistortion>(
        new RadTanDistortion(zeros, type));
  } else if (type == beam_calibration::DistortionType::EQUIDISTANT) {
    return std::unique_ptr<beam_calibration::EquidistantDistortion>(
        new EquidistantDistortion(coeffs, type));
  } else {
    return nullptr;
  }
}

void DistortionModel::SetCoefficients(beam::VecX coeffs) {
  if (coeffs.size() != get_size_[this->GetType()]) {
    LOG_ERROR("Invalid number of elements in coefficients vector.");
    throw std::runtime_error{
        "Invalid number of elements in coefficients vector."};
  } else {
    coefficients_ = coeffs;
    coefficients_valid_ = true;
  }
}

const beam::VecX DistortionModel::GetCoefficients() const {
  if (!coefficients_valid_) {
    LOG_ERROR("cannot retrieve coefficients, value not set.");
    throw std::runtime_error{"cannot retrieve coefficients, value not set"};
  }
  return coefficients_;
}

void DistortionModel::SetType(beam_calibration::DistortionType type) {
  type_ = type;
}

beam_calibration::DistortionType DistortionModel::GetType() {
  return type_;
}

} // namespace beam_calibration