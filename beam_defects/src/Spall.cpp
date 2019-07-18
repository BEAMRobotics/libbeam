#include "beam_defects/Spall.h"
#include "beam_defects/defect_functions.h"

namespace beam_defects {

double Spall::GetSize() {
  // Only calculate size first time this method is called
  if (!spall_size_) spall_size_ = CalculateSize();
  return spall_size_;
}

double Spall::CalculateSize() {
  if (defect_cloud_hull_->width == 0) {
    defect_cloud_hull_ = CalculateHull2D();
  }

  double spall_area = HullArea(defect_cloud_hull_);
  return spall_area;
}

DefectOSIMSeverity Spall::GetOSIMSeverity() {
  float largest_dimension = GetMaxDim2D();

  if (largest_dimension < 0.15) {
    return DefectOSIMSeverity::LIGHT;
  } else if (largest_dimension < 0.3) {
    return DefectOSIMSeverity::MEDIUM;
  } else if (largest_dimension < 0.6) {
    return DefectOSIMSeverity::SEVERE;
  } else {
    return DefectOSIMSeverity::VERY_SEVERE;
  }
}

} // namespace beam_defects
