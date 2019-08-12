#include "beam_defects/Defect.h"
#include "beam_defects/defect_functions.h"

#include <beam_utils/log.hpp>

#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/registration/icp.h>

#include <cmath>
namespace beam_defects {

// Common code will go here
Defect::Defect(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) {
  defect_cloud_ = pc;
  defect_cloud_initialized_ = true;
}

void Defect::SetPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr pc) {
  defect_cloud_ = pc;
  defect_cloud_initialized_ = true;
  BEAM_INFO("A new point cloud has been set.");
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Defect::GetPointCloud() {
  if (!defect_cloud_initialized_) {
    BEAM_WARN("Point cloud not initialized. GetPointCloud() returning empty "
              "point cloud");
  }
  return defect_cloud_;
}

void Defect::SetHullAlpha(float alpha) {
  alpha_ = alpha;
  cloud_hull_calculated_ = false;
  BEAM_INFO("Alpha hull value changed to {}.", alpha_);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Defect::GetHull2D() {
  // code that calculates the hull of a defect cloud
  auto calc_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto cloud_hull = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  if (!defect_cloud_initialized_) {
    BEAM_CRITICAL("Point cloud not initialized before call to GetHull2D().");
    throw std::runtime_error{"Point cloud not initialized."};
    return cloud_hull;
  }

  *calc_cloud = PCNoiseRemoval(defect_cloud_);
  *cloud_hull = ConcaveHull(calc_cloud, alpha_);
  std::vector<float> plane_norm_vect = PlaneNormalVector(cloud_hull);
  *calc_cloud = Project2Plane(cloud_hull, plane_norm_vect);

  cloud_hull_calculated_ = true;
  return cloud_hull;
};

std::vector<float> Defect::GetBBoxDims2D() {
  if (!cloud_hull_calculated_) { defect_cloud_hull_ = GetHull2D(); }

  std::vector<float> bounding_box(2, 0);
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D(*defect_cloud_hull_, minPt, maxPt);
  bounding_box[0] = maxPt.x - minPt.x;
  bounding_box[1] = maxPt.y - minPt.y;

  return bounding_box;
}

float Defect::GetMaxDim2D() {
  if (!cloud_hull_calculated_) { defect_cloud_hull_ = GetHull2D(); }

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

int Defect::GetMatchingDefect(const std::vector<Defect::Ptr>& defect_vector) {
  int ind = 0;
  int match_ind = 0;
  float match_dist = 100;

  for (auto& defect : defect_vector) {
    if (this->GetType() == defect->GetType()) {
      float hausdorff_dist =
          HausdorffDist(defect_cloud_, defect->GetPointCloud());
      if (hausdorff_dist < match_dist) {
        match_dist = hausdorff_dist;
        match_ind = ind;
      }
      ++ind;
    }
  }
  return match_ind;
}

} // namespace beam_defects
