#include "beam_defects/Corrosion.h"
#include "beam_defects/defect_functions.h"

namespace beam_defects {

Corrosion::Corrosion(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) : defect_cloud_(pc) {}

double Corrosion::GetSize() {
  // Only calculate size first time this method is called
  if (!corrosion_size_) corrosion_size_ = CalculateSize();
  return corrosion_size_;
}

double Corrosion::CalculateSize() {
  if (defect_cloud_->width == 0) return 0;

  // code that calculates the size of corrosion
  auto calc_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  *calc_cloud = PCNoiseRemoval(defect_cloud_);
  *calc_cloud = ConcaveHull(calc_cloud);
  std::vector<float> plane_norm_vect = PlaneNormalVector(calc_cloud);
  *calc_cloud = Project2Plane(calc_cloud, plane_norm_vect);
  double corrosion_area = HullArea(calc_cloud);

  return corrosion_area;
}

DefectOSIMSeverity Corrosion::GetOSIMSeverity() {
  double corrosion_area = GetSize();
  if (corrosion_area == 0) {
    return DefectOSIMSeverity::NONE;
  } else if (corrosion_area < 0.0225) {
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
