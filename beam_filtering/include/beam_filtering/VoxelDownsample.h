/** @file
 * @ingroup filtering
 */

#pragma once

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

#include <beam_filtering/Filter.h>

namespace beam_filtering {
/**
 * @addtogroup filtering
 */

/**
 * @brief VoxelDownsample is a filter for downsampling a pointcloud using a
 * voxel grid. Points in each voxel are replaced with a single point in their
 * centroid.  This is currently implemented as a wrapper over PCL's voxel grid
 * filter.
 *
 * PCL stores the number of voxels using 32bit integers, so integer overflow
 * protection is implemented. Clouds are split up to be filtered piecewise if
 * overflow is predicted. Midpoint splitting is used in a recursive pattern
 * until overflow is not predicted for any piece.  The output cloud is the
 * concatenation of each piece filtered.
 *
 * VoxelDownsample currently supports the PointXYZ point type with templating
 * coming soon.
 */
template <class PointT = pcl::PointXYZ>
class VoxelDownsample : public FilterBase<PointT> {
public:
  using PointCloudType = pcl::PointCloud<PointT>;
  using PointCloudTypePtr = std::shared_ptr<PointCloudType>;
  /**
   * @brief Constructor.
   * @param voxel_size Initial voxel size in x, y, and z.
   */
  VoxelDownsample(const Eigen::Vector3f& voxel_size = Eigen::Vector3f(0.05,
                                                                      0.05,
                                                                      0.05))
      : voxel_size_(voxel_size) {}

  /**
   * @brief Default destructor.
   */
  ~VoxelDownsample() = default;

  /**
   * @brief Get current voxel_size_.
   * @return Current voxel size.
   */
  inline Eigen::Vector3f GetVoxelSize() const { return voxel_size_; }

  /**
   * @brief Set a new voxel_size_.
   * @param voxel_size New voxel size.
   */
  inline void SetVoxelSize(const Eigen::Vector3f& voxel_size) {
    voxel_size_ = voxel_size;
  }

  /**
   * @brief Method for returning type of defect
   * @return filter type
   */
  inline FilterType GetType() const override { return FilterType::VOXEL; }

  /**
   * @brief Method for applying the filter to clouds
   * @return true if successful
   */
  inline bool Filter() override {
    // Clear points in output cloud
    this->output_cloud_.clear();

    // check cloud has points
    if (this->input_cloud_->size() == 0) { return false; }

    // Filter cloud in pieces to prevent integer overflow.
    std::vector<PointCloudTypePtr> broken_clouds =
        BreakUpPointCloud(*(this->input_cloud_));

    // Create a pcl voxel grid filter and use voxel_size_ as grid size.
    pcl::VoxelGrid<PointT> downsampler;
    downsampler.setLeafSize(voxel_size_[0], voxel_size_[1], voxel_size_[2]);
    for (const auto& cloud : broken_clouds) {
      PointCloudType downsampled_points;
      downsampler.setInputCloud(cloud);
      downsampler.filter(downsampled_points);
      this->output_cloud_ += downsampled_points;
    }

    return true;
  }

private:
  /**
   * @brief Private method for breaking up clouds.
   * @param input_cloud Reference to the cloud to be broken up.
   * @return A vector of the broken up clouds.
   */
  inline std::vector<PointCloudTypePtr>
      BreakUpPointCloud(const PointCloudType& input_cloud) {
    // Determine if integer overflow will occur.
    PointT min;
    PointT max;
    pcl::getMinMax3D(input_cloud, min, max);
    Eigen::Vector3f axis_dimensions(std::abs(max.x - min.x),
                                    std::abs(max.y - min.y),
                                    std::abs(max.z - min.z));
    uint64_t voxel_count_x =
        static_cast<uint64_t>((axis_dimensions[0] / voxel_size_[0]) + 1);
    uint64_t voxel_count_y =
        static_cast<uint64_t>((axis_dimensions[1] / voxel_size_[1]) + 1);
    uint64_t voxel_count_z =
        static_cast<uint64_t>((axis_dimensions[2] / voxel_size_[2]) + 1);

    if ((voxel_count_x * voxel_count_y * voxel_count_z) >
        static_cast<uint64_t>(std::numeric_limits<std::int32_t>::max())) {
      // Split cloud along max axis until integer overflow does not occur.
      int max_axis;
      axis_dimensions.maxCoeff(&max_axis);
      std::pair<PointCloudTypePtr, PointCloudTypePtr> split_clouds =
          SplitCloudInTwo(input_cloud, max_axis);

      // Recursively call BreakUpPointCloud on each split.
      std::vector<PointCloudTypePtr> cloud_1 =
          BreakUpPointCloud(*(split_clouds.first));
      std::vector<PointCloudTypePtr> cloud_2 =
          BreakUpPointCloud(*(split_clouds.second));
      cloud_1.insert(cloud_1.end(), cloud_2.begin(), cloud_2.end());
      return cloud_1;
    } else {
      // No overflow, return output cloud pointers in vector.
      PointCloudTypePtr output_cloud =
          std::make_shared<PointCloudType>(input_cloud);
      return std::vector<PointCloudTypePtr>{output_cloud};
    }
  }

  /**
   * @brief Private method for splitting one cloud into two
   * format.
   * @param input_cloud Cloud to be split.
   * @param max_axis The axis of greatest magnitude to split the cloud.
   * @return A pair of the split clouds.
   */
  inline std::pair<PointCloudTypePtr, PointCloudTypePtr>
      SplitCloudInTwo(const PointCloudType& input_cloud, int max_axis) {
    // Get mid point of cloud.
    PointT min;
    PointT max;
    pcl::getMinMax3D(input_cloud, min, max);
    Eigen::Vector3f eigen_min(min.x, min.y, min.z);
    Eigen::Vector3f eigen_max(max.x, max.y, max.z);
    float midpoint = (eigen_max[max_axis] + eigen_min[max_axis]) / 2;

    // Create clouds for points (1) < than midpoint (2) > than midpoint.
    PointCloudTypePtr cloud_1 = std::make_shared<PointCloudType>();
    PointCloudTypePtr cloud_2 = std::make_shared<PointCloudType>();
    for (const PointT& point : input_cloud) {
      Eigen::Vector3f eigen_point(point.x, point.y, point.z);
      if (eigen_point[max_axis] < midpoint) {
        cloud_1->push_back(point);
      } else {
        cloud_2->push_back(point);
      }
    }

    return std::pair<PointCloudTypePtr, PointCloudTypePtr>(cloud_1, cloud_2);
  }

  Eigen::Vector3f voxel_size_{0.05, 0.05, 0.05};
};

} // namespace beam_filtering