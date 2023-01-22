/** @file
 * @ingroup filtering
 */

#pragma once

#include <beam_filtering/Filter.h>

namespace beam_filtering {
/// @addtogroup filtering

/**
 * @brief class for radius outlier removal filter. See PCL docs
 */
template <class PointT = pcl::PointXYZ>
class ROR : public FilterBase<PointT> {
public:
  using PointCloudType = pcl::PointCloud<PointT>;
  using PointCloudTypePtr = std::shared_ptr<PointCloudType>;

  /**
   * @brief Constructor
   * @param radius_search
   * @param min_neighbors minimum number of neighbors to classify the point as
   * an inline
   */
  ROR(float radius_search = 0.1, int min_neighbors = 5)
      : radius_search_(radius_search), min_neighbors_(min_neighbors) {}

  /**
   * @brief Default destructor
   */
  ~ROR() = default;

  /**
   * @brief Method for setting radius multiplier
   * @param radius_multiplier
   */
  inline void SetRadiusSearch(float radius_search) {
    radius_search_ = radius_search;
  }

  /**
   * @brief Method for retrieving radius multiplier
   * @return radius_multiplier default = 3. This should be greater than 1
   */
  inline float GetRadiusSearch() const { return radius_search_; }

  /**
   * @brief Method for setting minimum number of neighbors
   * @param min_neighbors
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
   * @brief Method for returning type of defect
   * @return filter type
   */
  inline FilterType GetType() const override { return FilterType::ROR; }

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
    beam::KdTree<PointT> kd_tree(*this->input_cloud_);

    // Go over all the points and check which doesn't have enough neighbors
    // perform filtering
    for (auto p = this->input_cloud_->begin(); p != this->input_cloud_->end();
         p++) {
      std::vector<uint32_t> point_id_radius_search;
      std::vector<float> point_radius_squared_dist;
      size_t neighbors =
          kd_tree.radiusSearch(*p, radius_search_, point_id_radius_search,
                               point_radius_squared_dist);
      if (neighbors >= min_neighbors_) { this->output_cloud_.push_back(*p); }
    }

    return true;
  }

private:
  float radius_search_{0.1};
  float min_neighbors_{5};
};

} // namespace beam_filtering
