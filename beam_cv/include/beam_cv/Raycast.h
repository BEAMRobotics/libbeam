/** @file
 * @ingroup depth
 */

#pragma once

#include <beam_depth/Utils.h>
#include <beam_utils/utils.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <beam_calibration/CameraModel.h>

namespace beam_cv {

template <typename PointType>
class Raycast {
public:
  /**
   * @brief Custom constructor
   */
  Raycast(typename pcl::PointCloud<PointType>::Ptr cloud_i,
          std::shared_ptr<beam_calibration::CameraModel> model_i,
          std::shared_ptr<cv::Mat> image_i)
      : cloud_(cloud_i), model_(model_i), image_(image_i) {}

  /**
   * @brief Performs ray casting with custom behaviour on hit
   * @param dilation for area around point to search
   * @param threshold threshold to determine ray contact (how far between ray
   * tip and point is considered a hit in metres)
   * @param behaviour function that determines the hit behaviour
   */
  template <typename func>
  void Execute(int dilation, float threshold, func behaviour) {
    BEAM_INFO("Performing ray casting.");
    cv::Mat hit_mask = CreateHitMask(dilation);
    /// create image with 3 channels for coordinates
    PointType origin(0, 0, 0);
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(cloud_);
    /// Compute depth image with point cloud
    image_->forEach<float>([&](float& pixel, const int* position) -> void {
      (void)pixel;
      int row = position[0], col = position[1];
      if (hit_mask.at<cv::Vec3b>(row, col).val[0] == 255) {
        Eigen::Vector3d ray(0, 0, 0);
        // get direction vector
        Eigen::Vector2i input_point(col, row);
        opt<Eigen::Vector3d> point = model_->BackProject(input_point);
        if (!point.has_value()) { return; }
        // while loop to ray trace
        uint16_t raypt = 0;
        while (true) {
          // get point at end of ray
          PointType search_point;
          search_point.x = ray(0, 0);
          search_point.y = ray(1, 0);
          search_point.z = ray(2, 0);
          // search for closest point to ray
          std::vector<int> point_idx(1);
          std::vector<float> point_distance(1);
          kdtree.nearestKSearch(search_point, 1, point_idx, point_distance);
          float distance = sqrt(point_distance[0]);
          // if the point is within 1cm then color it appropriately
          if (distance < threshold) {
            behaviour(image_, cloud_, position, point_idx[0]);
            break;
          } else if (raypt >= 20) {
            break;
          } else {
            raypt++;
            ray(0, 0) = ray(0, 0) + distance * point.value()(0, 0);
            ray(1, 0) = ray(1, 0) + distance * point.value()(1, 0);
            ray(2, 0) = ray(2, 0) + distance * point.value()(2, 0);
          }
        }
      }
    });
    cv::Mat vis = beam_depth::VisualizeDepthImage(*image_);
    cv::imwrite("/home/jake/depth.jpg", vis);
  }

  /**
   * @brief Default destructor
   */
  ~Raycast() = default;

protected:
  typename pcl::PointCloud<PointType>::Ptr cloud_;
  std::shared_ptr<beam_calibration::CameraModel> model_;
  std::shared_ptr<cv::Mat> image_;

  /**
   * @brief Creates hit mask to speed up ray casting
   * @param mask_size size of mask used for dilating hit mask
   * @param model camera model used for projecting points
   * @param cloud cloud to use for hit detection
   * @return cv::Mat
   */
  cv::Mat CreateHitMask(int mask_size) {
    // create image mask where white pixels = projection hit
    cv::Size image_s(model_->GetHeight(), model_->GetWidth());
    cv::Mat tmp = cv::Mat::zeros(image_s, CV_8UC3);
    cv::Mat hit_mask;
    for (uint32_t i = 0; i < cloud_->points.size(); i++) {
      Eigen::Vector3d point;
      point << cloud_->points[i].x, cloud_->points[i].y, cloud_->points[i].z;
      opt<Eigen::Vector2i> coords = model_->ProjectPoint(point);
      if (!coords.has_value()) { continue; }
      uint16_t col = coords.value()(0, 0), row = coords.value()(1, 0);
      tmp.at<cv::Vec3b>(row, col).val[0] = 255;
    }
    cv::dilate(tmp, hit_mask, cv::Mat(mask_size, mask_size, CV_8UC1),
               cv::Point(-1, -1), 1, 1, 1);
    return hit_mask;
  }
};
} // namespace beam_cv
