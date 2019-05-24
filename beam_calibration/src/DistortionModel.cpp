#include "beam_calibration/DistortionModel.h"
#include "beam_calibration/EquidistantDistortion.h"
#include "beam_calibration/RadTanDistortion.h"

namespace beam_calibration {

std::shared_ptr<DistortionModel> DistortionModel::Create(DistortionType type,
                                                         beam::VecX coeffs) {
  if (type == DistortionType::RADTAN) {
    return std::shared_ptr<RadTanDistortion>(new RadTanDistortion(coeffs));
  } else if (type == DistortionType::NONE) {
    return std::shared_ptr<RadTanDistortion>(
        new RadTanDistortion(beam::VecX::Zero(5)));
  } else if (type == DistortionType::EQUIDISTANT) {
    return std::shared_ptr<EquidistantDistortion>(
        new EquidistantDistortion(coeffs));
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

DistortionType DistortionModel::GetType() {
  return type_;
}

} // namespace beam_calibration