#include "beam_defects/Spall.h"
#include "beam_defects/defect_functions.h"

namespace beam_defects {

Spall::Spall(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) : defect_cloud_(pc) {}

double Spall::GetSize() {
  // Only calculate size first time this method is called
  if (!spall_size_) spall_size_ = CalculateSize();
  return spall_size_;
}

double Spall::CalculateSize() {
  if (defect_cloud_->width == 0) return 0;

  // code that calculates the area of a spall
  auto calc_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  *calc_cloud = PCNoiseRemoval(defect_cloud_);
  *calc_cloud = calculateHull(calc_cloud);
  std::vector<float> plane_norm_vect = planeNormalVector(calc_cloud);
  *calc_cloud = project2Plane(calc_cloud, plane_norm_vect);
  double spall_area = calculateHullArea(calc_cloud);

  return spall_area;
}

DefectOSIMSeverity Spall::GetOSIMSeverity(){
  double spall_area = GetSize();
  if (spall_area == 0) {
    return DefectOSIMSeverity::NONE;
  } else if (spall_area < 0.0225) {
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
