#include "beam_defects/Defect.h"
#include "beam_defects/defect_functions.h"

namespace beam_defects {

// Common code will go here
Defect::Defect(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) {
  defect_cloud_ = pc;
  point_cloud_initialized_ = true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Defect::CalculateHull2D(){
  // code that calculates the hull of a defect cloud
  auto calc_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto cloud_hull = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  *calc_cloud = PCNoiseRemoval(defect_cloud_);
  *cloud_hull = ConcaveHull(calc_cloud);
  std::vector<float> plane_norm_vect = PlaneNormalVector(cloud_hull);
  *calc_cloud = Project2Plane(cloud_hull, plane_norm_vect);
  
  return cloud_hull;
};

// std::vector<float> Defect::GetBoundingBox2D(){
//   if (defect_cloud_hull_->width == 0) return 0;

// }

}
