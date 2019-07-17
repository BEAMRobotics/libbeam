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
  std::vector<float> bounding_box = GetBoundingBox2D();
  float largest_dimension;
  if (bounding_box[2] > bounding_box[3]) {
    largest_dimension = bounding_box[2];
  } else {
    largest_dimension = bounding_box[3];
  }

  if (largest_dimension < 0.15) {
    return DefectOSIMSeverity::LIGHT;
  } else if (largest_dimension < 0.3) {
    return DefectOSIMSeverity::MEDIUM;
  } else if (largest_dimension < 0.6 ) {
    return DefectOSIMSeverity::SEVERE;
  } else {
    return DefectOSIMSeverity::VERY_SEVERE;
  }
  // double delam_area = GetSize();
  // if (delam_area < 0.0225 ) {
  //   return DefectOSIMSeverity::LIGHT;
  // } else if (delam_area < 0.09) {
  //   return DefectOSIMSeverity::MEDIUM;
  // } else if (delam_area < 0.36) {
  //   return DefectOSIMSeverity::SEVERE;
  // } else {
  //   return DefectOSIMSeverity::VERY_SEVERE;
  // }
}

} // namespace beam_defects
