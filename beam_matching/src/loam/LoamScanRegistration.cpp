#include <beam_matching/loam/LoamScanRegistration.h>

#include <Eigen/Geometry>

#include <beam_utils/log.h>

namespace beam_matching {

LoamScanRegistration::LoamScanRegistration(const LoamParamsPtr& params)
    : params_(params) {}

} // namespace beam_matching
