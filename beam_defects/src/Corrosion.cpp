#include "beam_defects/Corrosion.h"
#include "beam_defects/defect_functions.h"

namespace beam_defects {

double Corrosion::GetSize() {
  // Only calculate size first time this method is called
  if (!corrosion_size_) corrosion_size_ = CalculateSize();
  return corrosion_size_;
}

double Corrosion::CalculateSize() {
  if (defect_cloud_hull_->width==0){
    defect_cloud_hull_ = GetHull2D();
  }
  
  double corrosion_area = HullArea(defect_cloud_hull_);
  return corrosion_area;
}

DefectOSIMSeverity Corrosion::GetOSIMSeverity() {
  double corrosion_area = GetSize();
  if (corrosion_area < 0.0225) {
    return DefectOSIMSeverity::LIGHT;
  } else if (corrosion_area < 0.09) {
    return DefectOSIMSeverity::MEDIUM;
  } else if (corrosion_area < 0.36) {
    return DefectOSIMSeverity::SEVERE;
  } else {
    return DefectOSIMSeverity::VERY_SEVERE;
  }
}

} // namespace beam_defects
