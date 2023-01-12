/** @file
 * @ingroup filtering
 */

#pragma once

#include <math.h>

#include <beam_utils/kdtree.h>
#include <pcl/filters/extract_indices.h>

#include <beam_filtering/Filter.h>

namespace beam_filtering {
/// @addtogroup filtering

/**
 * @brief class for dynamic radius outlier removal filter. See paper:
 * https://ieeexplore.ieee.org/abstract/document/8575761
 */
template <class PointT = pcl::PointXYZ>
class DROR : public FilterBase<PointT> {
public:
  using PointCloudType = pcl::PointCloud<PointT>;
  using PointCloudTypePtr = std::shared_ptr<PointCloudType>;

  /**
   * @brief Constructor
   * @param radius_multiplier the final search radius is multiplied by this
   * parameter to add or remove some cussion. Tune this param to over or under
   * filter.
   * @param azimuth_angle azimuth angle of the lidar IN DEGREES. In other words,
   * what is the rotation of the beam for each lidar pulse. This is used to
   * estimate the density of the points at any distance
   * @param min_neighbors minimum number of neighbors to classify the point as
   * an inline. Note: this number includes the point that is being filtered (3
   * means 2 neighbors)
   * @param min_search_radius set a minimum search radius for points close up
   */
  DROR(float radius_multiplier = 3, float azimuth_angle = 0.04,
       int min_neighbors = 3, float min_search_radius = 0.04)
      : radius_multiplier_(radius_multiplier),
        azimuth_angle_(azimuth_angle),
        min_neighbors_(min_neighbors),
        min_search_radius_(min_search_radius) {}

  /**
   * @brief Default destructor
   */
  ~DROR() = default;

  /**
   * @brief Method for setting radius multiplier
   * @param radius_multiplier
   */
  inline void SetRadiusMultiplier(float radius_multiplier) {
    radius_multiplier_ = radius_multiplier;
  }

  /**
   * @brief Method for retrieving radius multiplier
   * @return radius_multiplier default = 3. This should be greater than 1
   */
  inline float GetRadiusMultiplier() const { return radius_multiplier_; }

  /**
   * @brief Method for setting azimuth angle
   * @param azimuth_angle default = 0.04 degees
   */
  inline void SetAzimuthAngle(float azimuth_angle) {
    azimuth_angle_ = azimuth_angle;
  }

  /**
   * @brief Method for getting azimuth angle in degrees
   * @return azimuth_angle
   */
  inline float GetAzimuthAngle() const { return azimuth_angle_; }

  /**
   * @brief Method for setting minimum number of neighbors which includes the
   * point that is being filtered (3 means 2 neighbors)
   * @param min_neighbors default = 3
   */
  inline void SetMinNeighbors(int min_neighbors) {
    min_neighbors_ = min_neighbors;
  }

  /**
   * @brief Method for retrieving minimum number of neighbors
   * @return min_neighbors
   */
  inline int GetMinNeighbors() const { return min_neighbors_; }

  /**
   * @brief Method for setting the minimum search radius
   * @param min_search_radius default = 0.04
   */
  inline void SetMinSearchRadius(float min_search_radius) {
    min_search_radius_ = min_search_radius;
  }

  /**
   * @brief Method for retrieving the minimum search radius
   * @return min_search_radius
   */
  inline float GetMinSearchRadius() const { return min_search_radius_; }

  /**
   * @brief Method for returning type of defect
   * @return filter type
   */
  inline FilterType GetType() const override { return FilterType::DROR; }

  /**
   * @brief Method for applying the dror filter
   * @return true if successful
   */
  inline bool Filter() override {
    // Clear points in output cloud
    this->output_cloud_.clear();

    // check cloud has points
    if (this->input_cloud_->size() == 0) { return false; }

    // init. kd search tree
    beam::nanoflann::KdTree<pcl::PointXYZ> kd_tree;
    kd_tree.setInputCloud(this->input_cloud_);

    // Go over all the points and check which doesn't have enough neighbors
    // perform filtering
    for (auto p = this->input_cloud_->begin(); p != this->input_cloud_->end();
         p++) {
      float range_i = sqrt(pow(p->x, 2) + pow(p->y, 2));
      float search_radius_dynamic =
          radius_multiplier_ * azimuth_angle_ * M_PI / 180 * range_i;

      if (search_radius_dynamic < min_search_radius_) {
        search_radius_dynamic = min_search_radius_;
      }

      std::vector<int> point_id_radius_search;
      std::vector<float> point_radius_squared_dist;

      int neighbors = kd_tree.radiusSearch(*p, search_radius_dynamic,
                                           point_id_radius_search,
                                           point_radius_squared_dist);

      if (neighbors >= min_neighbors_) { this->output_cloud_.push_back(*p); }
    }

    return true;
  }

private:
  float radius_multiplier_{3};
  float azimuth_angle_{0.04};
  float min_neighbors_{3};
  float min_search_radius_{0.04};
};

} // namespace beam_filtering
