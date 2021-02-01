/** @file
 * @ingroup depth
 */

#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <beam_calibration/CameraModel.h>
#include <beam_containers/PointBridge.h>
#include <beam_depth/Utils.h>
#include <beam_utils/utils.h>

#include <functional>
#include <type_traits>

namespace beam_cv {
/**
 * @brief Class to store a raycasting object for a specified point cloud
 * and camera model.
 *
 * Valid point types:
 * - [pcl::PointXYZ]
 * - [pcl::PointZXYZRGB],
 * - [beam_containers::PointBridge]
 */

template <typename PointType>
class Raycast {
public:
  /**
   * @brief Constructor for initializing with cloud, camera model and image
   */
  Raycast(typename pcl::PointCloud<PointType>::Ptr cloud_i,
          std::shared_ptr<beam_calibration::CameraModel> model_i,
          std::shared_ptr<cv::Mat> image_i)
      : cloud_(cloud_i), model_(model_i), image_(image_i) {}

  /**
   * @brief Default destructor
   */
  ~Raycast() = default;

  /**
   * @brief Performs ray casting with custom behaviour on hit
   * @param threshold threshold to determine ray contact (how far between ray
   * tip and point is considered a hit in metres)
   * @param behaviour function that determines the hit behaviour, expected
   signature:
   * {std::shared_ptr<cv::Mat>& image,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                     const int* position, int index}
   */
  template <typename func>
  void Execute(float threshold, func behaviour) {
    BEAM_INFO("Performing ray casting.");
    // create copied point cloud to use for kdtree
    pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud =
        boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (size_t i = 0; i < cloud_->points.size(); i++) {
      pcl::PointXYZ p(cloud_->points[i].x, cloud_->points[i].y,
                      cloud_->points[i].z);
      template_cloud->points.push_back(p);
    }
    // create image mask where white pixels = projection hit
    cv::Mat1b hit_mask(model_->GetHeight(), model_->GetWidth());
    for (uint32_t i = 0; i < cloud_->points.size(); i++) {
      Eigen::Vector3d point(cloud_->points[i].x, cloud_->points[i].y,
                            cloud_->points[i].z);

      beam::opt<Eigen::Vector2i> coords = model_->ProjectPoint(point);
      if (!coords.has_value()) { continue; }
      uint16_t col = coords.value()(0, 0);
      uint16_t row = coords.value()(1, 0);
      hit_mask.at<uchar>(row, col) = 255;
    }
    // create kdtree
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(template_cloud);
    // cast ray for every pixel
    for (int row = 0; row < image_->rows; row++) {
      for (int col = 0; col < image_->cols; col++) {
        if (hit_mask.at<uchar>(row, col) == 255) {
          Eigen::Vector3d ray(0, 0, 0);
          // get direction vector
          Eigen::Vector2i input_point(col, row);
          beam::opt<Eigen::Vector3d> point = model_->BackProject(input_point);
          if (!point.has_value()) { return; }
          // while loop to ray trace
          uint16_t raypt = 0;
          while (raypt <= 20) {
            // get point at end of ray
            pcl::PointXYZ search_point;
            search_point.x = ray(0, 0);
            search_point.y = ray(1, 0);
            search_point.z = ray(2, 0);
            // search for closest point to ray
            std::vector<int> point_idx(1);
            std::vector<float> point_distance(1);
            kdtree.nearestKSearch(search_point, 1, point_idx, point_distance);
            float distance = sqrt(point_distance[0]);
            // if the point is within threshold then call behaviour
            if (distance < threshold) {
              int position[2];
              position[0] = row;
              position[1] = col;
              behaviour(image_, cloud_, position, point_idx[0]);
              break;
            } else {
              raypt++;
              ray(0, 0) = ray(0, 0) + distance * point.value()(0, 0);
              ray(1, 0) = ray(1, 0) + distance * point.value()(1, 0);
              ray(2, 0) = ray(2, 0) + distance * point.value()(2, 0);
            }
          }
        }
      }
    }
  }

  /**
   * @brief Cloud setter
   * @param cloud_i cloud to set
   * @return void
   */
  void SetCloud(typename pcl::PointCloud<PointType>::Ptr cloud_i) {
    cloud_ = cloud_i;
  }

  /**
   * @brief Camera model setter
   * @param model_i camera model to set
   * @return void
   */
  void SetCameraModel(std::shared_ptr<beam_calibration::CameraModel> model_i) {
    model_ = model_i;
  }

  /**
   * @brief Image setter
   * @param image_i image to set
   * @return void
   */
  void SetImage(std::shared_ptr<cv::Mat> image_i) { image_ = image_i; }

  /**
   * @brief Cloud getter
   * @return point cloud
   */
  typename pcl::PointCloud<PointType>::Ptr GetCloud() { return cloud_; }

  /**
   * @brief Image getter
   * @return image
   */
  std::shared_ptr<cv::Mat> GetImage() { return image_; }

protected:
  typename pcl::PointCloud<PointType>::Ptr cloud_;
  std::shared_ptr<beam_calibration::CameraModel> model_;
  std::shared_ptr<cv::Mat> image_;
};
} // namespace beam_cv
