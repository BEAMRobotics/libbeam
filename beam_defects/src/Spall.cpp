#include "beam_defects/Spall.h"
#include "beam_defects/defect_functions.h"

namespace beam_defects {

double Spall::GetSize() {
  // Only calculate size first time this method is called
  if (!spall_size_) spall_size_ = CalculateSize();
  return spall_size_;
}

double Spall::CalculateSize() {
  if (defect_cloud_hull_->width==0){
    defect_cloud_hull_ = CalculateHull2D();
  }
  
  double spall_area = HullArea(defect_cloud_hull_);
  return spall_area;
}

DefectOSIMSeverity Spall::GetOSIMSeverity(){
  double spall_area = GetSize();
  if (spall_area < 0.0225) {
    return DefectOSIMSeverity::LIGHT;
  } else if (spall_area < 0.09) {
    return DefectOSIMSeverity::MEDIUM;
  } else if (spall_area < 0.36) {
    return DefectOSIMSeverity::SEVERE;
  } else {
    return DefectOSIMSeverity::VERY_SEVERE;
  }
}

} // namespace beam_defects
