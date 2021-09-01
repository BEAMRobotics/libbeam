/** @file
 * @ingroup filtering
 */

#pragma once

#include <pcl/common/transforms.h>

#include <beam_filtering/Filter.h>

namespace beam_filtering {
/// @addtogroup filtering

/**
 * @brief class for crop box filter
 */
template <class PointT = pcl::PointXYZ>
class CropBox : public FilterBase<PointT> {
public:
  using PointCloudType = pcl::PointCloud<PointT>;
  using PointCloudTypePtr = std::shared_ptr<PointCloudType>;

  /**
   * @brief constructor which sets some defaults
   * @param min_vec all points greater than these coordinates (x, y, z) will be
   * considered part of the cropbox. These are expressed in the cropbox
   * coordinate, if T_box_cloud is set, these coordinates will be transformed
   * accordingly.
   * @param max_vec all points less than these coordinates (x, y, z) will be
   * considered part of the cropbox. These are expressed in the cropbox
   * coordinate, if T_box_cloud is set, these coordinates will be transformed
   * accordingly.
   * @param T_box_cloud transform expressing pose of cropbox within the cloud
   * frame
   * @param remove_outside_points set to true to remove the points outside the
   * box defined by min/max vecs
   */
  CropBox(const Eigen::Vector3f& min_vec = Eigen::Vector3f(0, 0, 0),
          const Eigen::Vector3f& max_vec = Eigen::Vector3f(20, 20, 20),
          const Eigen::Affine3f& T_box_cloud =
              Eigen::Affine3f(Eigen::Matrix4f::Identity()),
          bool remove_outside_points = false)
      : min_vec_(min_vec),
        max_vec_(max_vec),
        T_box_cloud_(T_box_cloud),
        remove_outside_points_(remove_outside_points) {}

  /**
   * @brief Default destructor
   */
  ~CropBox() = default;

  /**
   * @brief Method for retrieving minimum vector [xmin, ymin, zmin]
   * @return min_vec
   */
  inline Eigen::Vector3f GetMinVector() const { return min_vec_; }

  /**
   * @brief Method for setting minimum vector [xmin, ymin, zmin]
   * @param min_vec
   */
  inline void SetMinVector(Eigen::Vector3f& min_vec) { min_vec_ = min_vec; }

  /**
   * @brief Method for retrieving maximum vector [xmax, ymax, zmax]
   * @return max_vec
   */
  inline Eigen::Vector3f GetMaxVector() const { return max_vec_; }

  /**
   * @brief Method for setting maximum vector [xmax, ymax, zmax]
   * @param max_vec
   */
  inline void SetMaxVector(Eigen::Vector3f& max_vec) { max_vec_ = max_vec; }

  /**
   * @brief Method for getting the transformation used to transform the cropbox
   * @return T_box_cloud (from cloud frame to box frame)
   */
  inline Eigen::Affine3f GetTransform() const { return T_box_cloud_; }

  /**
   * @brief Method for getting the transformation used to transform the cropbox
   * @return T_box_cloud (from cloud frame to box frame)
   */
  inline Eigen::Matrix4f GetTransformMatrix() const {
    return T_box_cloud_.matrix();
  }

  /**
   * @brief Method for applying a transform to the cropbox
   * @param T_box_cloud (from cloud frame to box frame)
   */
  inline void SetTransform(const Eigen::Affine3f& T_box_cloud) {
    T_box_cloud_ = T_box_cloud;
  }

  /**
   * @brief Method for applying a transform to the cropbox
   * @param T_box_cloud (from cloud frame to box frame)
   */
  inline void SetTransform(const Eigen::Matrix4f& T_box_cloud) {
    T_box_cloud_.matrix() = T_box_cloud;
  }

  /**
   * @brief Method for setting whether or not the points inside or outside the
   * box are removed.
   * @param remove_outside_points (default: true). True means it removes the
   * points outside the box
   */
  inline void SetRemoveOutsidePoints(bool remove_outside_points) {
    remove_outside_points_ = remove_outside_points;
  }

  /**
   * @brief Method for getting whether or not the points inside or outside the
   * box are removed.
   * @return remove_outside_points
   */
  inline bool GetRemoveOutsidePoints() const { return remove_outside_points_; }

  /**
   * @brief Method for returning type of defect
   * @return filter type
   */
  inline FilterType GetType() const override { return FilterType::CROPBOX; }

  /**
   * @brief Method for applying the cropbox
   * @return true if successful
   */
  inline bool Filter() override {
    // Clear points in output cloud
    this->output_cloud_.clear();

    // check cloud has points
    if (this->input_cloud_->size() == 0) { return false; }

    // perform filtering
    for (auto p = this->input_cloud_->begin(); p != this->input_cloud_->end();
         p++) {
      PointT point = pcl::transformPoint(p, T_box_cloud_);

      if (point.x < min_vec_[0] || point.y < min_vec_[1] ||
          point.z < min_vec_[2] || point.x > max_vec_[0] ||
          point.y > max_vec_[1] || point.z > max_vec_[2]) {
        if (remove_outside_points_) {
          continue;
        } else {
          this->output_cloud_.push_back(p);
        }
      } else {
        if (remove_outside_points_) {
          this->output_cloud_.push_back(p);
        } else {
          continue;
        }
      }
    }

    return true;
  }

private:

  Eigen::Vector3f min_vec_{0, 0, 0};
  Eigen::Vector3f max_vec_{20, 20, 20};
  Eigen::Affine3f T_box_cloud_{Eigen::Matrix4f::Identity()};
  bool remove_outside_points_{true};

};

} // namespace beam_filtering
