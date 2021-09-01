/** @file
 * @ingroup filtering
 */

#pragma once

#include <beam_utils/pointclouds.h>
#include <beam_utils/log.h>

namespace beam_filtering {
/// @addtogroup filtering

/**
 * @brief Enum class for different types of filters we have implemented
 */
enum class FilterType { CROPBOX = 0, DROR, ROR, VOXEL };

/**
 * @brief Base class to define the interface for all beam filters, and provide
 * some utility functions. Since we need to template the filtering based on
 * point type, we need to keep the Filter() function independent of point type.
 * Virtual functions cannot be templated.
 */
template <class PointT = pcl::PointXYZ>
class FilterBase {
public:
  /**
   * @brief default constructor
   */
  FilterBase() = default;

  /**
   * @brief Default destructor
   */
  ~FilterBase() = default;

  // alias for clarity
  using Ptr = std::shared_ptr<FilterBase>;

  /**
   * @brief Pure virtual method for returning type of defect
   * @return filter type
   */
  virtual FilterType GetType() const = 0;

  /**
   * @brief set input cloud to be filtered
   * @param cloud shared pointer to cloud to be filtered
   */
  inline void
      SetInputCloud(const std::shared_ptr<pcl::PointCloud<PointT>>& cloud) {
    input_cloud_ = cloud;
  }

  /**
   * @brief set input cloud to be filtered
   * @param cloud shared pointer to cloud to be filtered
   */
  inline pcl::PointCloud<PointT> GetFilteredCloud() {
    if (output_cloud_.empty()) {
      BEAM_WARN("Returning empty cloud, make sure you set the input cloud and "
                "called Filter() before getting result.");
    }
    return output_cloud_;
  }

  /**
   * @brief Method for applying the filter
   * @return true if successful
   */
  virtual bool Filter() = 0;

protected:
  std::shared_ptr<pcl::PointCloud<PointT>> input_cloud_{nullptr};
  pcl::PointCloud<PointT> output_cloud_;
};

} // namespace beam_filtering
