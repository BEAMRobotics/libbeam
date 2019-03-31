#include "beam_defects/Delam.h"
#include "beam_defects/defect_functions.h"

namespace beam_defects {

Delam::Delam(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) : defect_cloud_(pc) {}

double Delam::GetSize() {
  // Only calculate size first time this method is called
  if (!delam_size_) delam_size_ = CalculateSize();
  return delam_size_;
}

double Delam::CalculateSize() {
  if (defect_cloud_->width == 0) return 0;

  // code that calculates the size of a delam
  auto cloud_hull = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  *cloud_hull = calculateHull(defect_cloud_);
  std::vector<float> plane_norm_vect = planeNormalVector(cloud_hull);
  *cloud_hull = project2Plane(cloud_hull, plane_norm_vect);
  double delam_area = calculateHullArea(cloud_hull);

  return delam_area;
}

DefectOSIMSeverity Delam::GetOSIMSeverity() {
  double delam_area = GetSize();
  if (delam_area == 0) {
    return DefectOSIMSeverity::NONE;
  } else if (delam_area < 0.0225) {
    return DefectOSIMSeverity::LIGHT;
  } else if (delam_area < 0.09) {
    return DefectOSIMSeverity::MEDIUM;
  } else if (delam_area < 0.36) {
    return DefectOSIMSeverity::SEVERE;
  } else {
    return DefectOSIMSeverity::VERY_SEVERE;
  }
}

} // namespace beam_defects
