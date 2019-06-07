#include "beam_filtering/DROR.h"

namespace beam_filtering {

void DROR::SetRadiusMultiplier(double& radius_multiplier) {
  radius_multiplier_ = radius_multiplier;
}

double DROR::GetRadiusMultiplier() {
  return radius_multiplier_;
}

void DROR::SetAzimuthAngle(double& azimuth_angle) {
  azimuth_angle_ = azimuth_angle;
}

double DROR::GetAzimuthAngle() {
  return azimuth_angle_;
}

void DROR::SetMinNeighbors(double& min_neighbors) {
  min_neighbors_ = min_neighbors;
}

double DROR::GetMinNeighbors() {
  return min_neighbors_;
}

void DROR::SetMinSearchRadius(double& min_search_radius) {
  min_search_radius_ = min_search_radius;
}

double DROR::GetMinSearchRadius() {
  return min_search_radius_;
}

void DROR::Filter(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                  pcl::PointCloud<pcl::PointXYZ>& filtered_cloud) {
  // Clear points in output cloud
  filtered_cloud.clear();

  // init. kd search tree
  KdTreePtr kd_tree_(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  kd_tree_->setInputCloud(input_cloud);

  // Go over all the points and check which doesn't have enough neighbors
  // perform filtering
  for (pcl::PointCloud<pcl::PointXYZ>::iterator it = input_cloud->begin();
       it != input_cloud->end(); ++it) {
    float x_i = it->x;
    float y_i = it->y;
    float range_i = sqrt(pow(x_i, 2) + pow(y_i, 2));
    float search_radius_dynamic =
        radius_multiplier_ * azimuth_angle_ * 3.14159265359 / 180 * range_i;

    if (search_radius_dynamic < min_search_radius_) {
      search_radius_dynamic = min_search_radius_;
    }

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    int neighbors =
        kd_tree_->radiusSearch(*it, search_radius_dynamic, pointIdxRadiusSearch,
                               pointRadiusSquaredDistance);

    if (neighbors >= min_neighbors_) { filtered_cloud.push_back(*it); }
  }
}

} // namespace beam_filtering
