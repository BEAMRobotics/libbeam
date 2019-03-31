#include "beam_defects/Crack.h"

namespace beam_defects {

Crack::Crack(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) : defect_cloud_(pc) {}

double Crack::GetSize() {
  // Only calculate size first time this method is called
  if (!crack_size_) crack_size_ = CalculateSize();
  return crack_size_;
}

double Crack::CalculateSize() {
  // code that calculates the size of a crack

  return 1.5; // Placeholder
}

DefectOSIMSeverity Crack::GetOSIMSeverity(){
  // Placeholder
  return DefectOSIMSeverity::LIGHT;
}

} // namespace beam_defects
