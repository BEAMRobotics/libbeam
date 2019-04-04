#include "beam_defects/Crack.h"
#include "beam_defects/defect_functions.h"

namespace beam_defects {

Crack::Crack(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) : defect_cloud_(pc) {}

double Crack::GetSize() {
  // Only calculate size first time this method is called
  if (!crack_size_) crack_size_ = CalculateSize();
  return crack_size_;
}

double Crack::CalculateSize() {
  if (defect_cloud_->width == 0) return 0;

  // code that calculates the size of a crack
  auto cloud_hull = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  *cloud_hull = calculateHull(defect_cloud_);
  std::vector<float> plane_norm_vect = planeNormalVector(cloud_hull);
  *cloud_hull = project2Plane(cloud_hull, plane_norm_vect);
  double crack_length = calculateMaxLength(cloud_hull);

  return crack_length;
}

DefectOSIMSeverity Crack::GetOSIMSeverity(){
  // Placeholder
  return DefectOSIMSeverity::LIGHT;
}

} // namespace beam_defects
