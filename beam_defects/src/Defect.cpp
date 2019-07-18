#include "beam_defects/Defect.h"
#include "beam_defects/defect_functions.h"

#include <beam_utils/log.hpp>

#include <pcl/common/common.h>
#include <pcl/common/distances.h>
namespace beam_defects {

// Common code will go here
Defect::Defect(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) {
  defect_cloud_ = pc;
  defect_cloud_initialized_ = true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Defect::CalculateHull2D() {
  // code that calculates the hull of a defect cloud
  auto calc_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto cloud_hull = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  if (!defect_cloud_initialized_) {
    throw std::runtime_error{"Point cloud not initialized."};
    BEAM_CRITICAL("Point cloud not initialized.");
    return cloud_hull;
  }

  *calc_cloud = PCNoiseRemoval(defect_cloud_);
  *cloud_hull = ConcaveHull(calc_cloud);
  std::vector<float> plane_norm_vect = PlaneNormalVector(cloud_hull);
  *calc_cloud = Project2Plane(cloud_hull, plane_norm_vect);

  return cloud_hull;
};

std::vector<float> Defect::GetBBoxDims2D() {
  if (defect_cloud_hull_->width == 0) {
    defect_cloud_hull_ = CalculateHull2D();
  }

  std::vector<float> bounding_box(2, 0);
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D(*defect_cloud_hull_, minPt, maxPt);
  bounding_box[0] = maxPt.x - minPt.x;
  bounding_box[1] = maxPt.y - minPt.y;

  return bounding_box;
}

float Defect::GetMaxDim2D() {
  if (defect_cloud_hull_->width == 0) {
    defect_cloud_hull_ = CalculateHull2D();
  }
  auto mock_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  *mock_cloud = *defect_cloud_hull_;
  float max_dist = 0;

  for (unsigned int i = 0; i < defect_cloud_hull_->size(); ++i) {
    for (auto& point : mock_cloud->points) {
      float dist = pcl::euclideanDistance(defect_cloud_hull_->points[i], point);
      if (dist > max_dist) { max_dist = dist; }
    }
  }
  return max_dist;
}

} // namespace beam_defects
