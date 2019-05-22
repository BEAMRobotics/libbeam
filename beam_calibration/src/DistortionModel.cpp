#include "beam_calibration/DistortionModel.h"
#include "beam_calibration/LadybugDistortion.h"
#include "beam_calibration/RadTanDistortion.h"

namespace beam_calibration {
std::unique_ptr<DistortionModel> DistortionModel::Create(DistortionType type,
                                                         beam::VecX coeffs,
                                                         unsigned int cam_id) {
  if (type == beam_calibration::DistortionType::RADTAN) {
    return std::unique_ptr<beam_calibration::RadTanDistortion>(
        new RadTanDistortion(coeffs));
  } else if (type == beam_calibration::DistortionType::NONE) {
    beam::VecX zeros = beam::VecX::Zero(5);
    return std::unique_ptr<beam_calibration::RadTanDistortion>(
        new RadTanDistortion(zeros));
  } else if (type == beam_calibration::DistortionType::LADYBUG) {
    return std::unique_ptr<beam_calibration::LadybugDistortion>(
        new LadybugDistortion(cam_id));
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