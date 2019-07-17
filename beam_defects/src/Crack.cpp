#include "beam_defects/Crack.h"
#include "beam_defects/defect_functions.h"

namespace beam_defects {

double Crack::GetSize() {
  // Only calculate size first time this method is called
  if (!crack_size_) crack_size_ = CalculateSize();
  return crack_size_;
}

double Crack::CalculateSize() {
  if (defect_cloud_hull_->width==0){
    defect_cloud_hull_ = CalculateHull2D();
  }

  double crack_length = MaxLength(defect_cloud_hull_);
  return crack_length;
}

DefectOSIMSeverity Crack::GetOSIMSeverity(){
  // Placeholder
  return DefectOSIMSeverity::LIGHT;
}

} // namespace beam_defects
