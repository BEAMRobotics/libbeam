#include "beam_calibration/DistortionModel.h"
#include "beam_calibration/EquidistantDistortion.h"
#include "beam_calibration/RadTanDistortion.h"

namespace beam_calibration {

DistortionModel::DistortionModel(beam::VecX coeffs, DistortionType type) {
  this->SetType(type);
  this->SetCoefficients(coeffs);
}

std::shared_ptr<DistortionModel> DistortionModel::Create(DistortionType type,
                                                         beam::VecX coeffs) {
  if (type == DistortionType::RADTAN) {
    return std::shared_ptr<RadTanDistortion>(
        new RadTanDistortion(coeffs, type));
  } else if (type == DistortionType::NONE) {
    beam::VecX zeros = beam::VecX::Zero(5);
    return std::shared_ptr<RadTanDistortion>(new RadTanDistortion(zeros, type));
  } else if (type == DistortionType::EQUIDISTANT) {
    return std::shared_ptr<EquidistantDistortion>(
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

void DistortionModel::SetType(DistortionType type) {
  type_ = type;
}

DistortionType DistortionModel::GetType() {
  return type_;
}

} // namespace beam_calibration