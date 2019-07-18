#include "beam_defects/Delam.h"
#include "beam_defects/defect_functions.h"

namespace beam_defects {

double Delam::GetSize() {
  // Only calculate size first time this method is called
  if (!delam_size_) delam_size_ = CalculateSize();
  return delam_size_;
}

double Delam::CalculateSize() {
  if (defect_cloud_hull_->width == 0) {
    defect_cloud_hull_ = CalculateHull2D();
  }

  double delam_area = HullArea(defect_cloud_hull_);
  return delam_area;
}

DefectOSIMSeverity Delam::GetOSIMSeverity() {
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
