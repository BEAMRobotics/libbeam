#include "beam_calibration/Intrinsics.h"
#include "beam_calibration/Ladybug.h"
#include "beam_calibration/Fisheye.h"
#include "beam_calibration/Pinhole.h"

#include <memory>

namespace beam_calibration {

std::unique_ptr<Intrinsics> Intrinsics::Create(IntrinsicsType type,
                                               unsigned int id) {
  if (type == IntrinsicsType::LADYBUG) return std::make_unique<Ladybug>(id);
}

std::unique_ptr<Intrinsics> Intrinsics::Create(IntrinsicsType type) {
  if (type == IntrinsicsType::PINHOLE)
    return std::unique_ptr<Pinhole>(new Pinhole());
  else if (type == IntrinsicsType::LADYBUG)
    return nullptr;
  else if (type == IntrinsicsType::FISHEYE)
    return nullptr;
  else return nullptr;
}

} // namespace beam_calibration
