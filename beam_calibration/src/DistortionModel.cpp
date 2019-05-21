#include "beam_calibration/DistortionModel.h"
#include "beam_calibration/RadTanDistortion.h"

namespace beam_calibration {
std::unique_ptr<DistortionModel> DistortionModel::Create(DistortionType type,
                                                         beam::VecX coeffs) {
  if (type == beam_calibration::DistortionType::RADTAN) {
    return std::unique_ptr<beam_calibration::RadTanDistortion>(
        new RadTanDistortion(coeffs));
  } else {
    return nullptr;
  }
}

void DistortionModel::SetCoefficients(beam::VecX coeffs) {
  coefficients_ = coeffs;
}

const beam::VecX DistortionModel::GetCoefficients() const {
  return coefficients_;
}

} // namespace beam_calibration