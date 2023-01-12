/** @file
 * @ingroup depth
 */

#pragma once

#include <functional>
#include <type_traits>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <beam_calibration/CameraModel.h>
#include <beam_containers/PointBridge.h>
#include <beam_utils/pointcloud_.h>
#include <beam_utils/utils.h>

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
          std::shared_ptr<cv::Mat> image_i, int max_ray_extensions = 20)
      : cloud_(cloud_i),
        model_(model_i),
        image_(image_i),
        max_ray_extensions_(max_ray_extensions) {}

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
    // create a pointcloud with only points that have projected into the image
    pcl::PointCloud<pcl::PointXYZ>::Ptr search_cloud =
        std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    // each entry of this vec contains the associated original point id
    std::vector<int> search_cloud_pt_to_orig_cloud_pt;

    // create image mask where white pixels = projection hit
    BEAM_DEBUG("creating hit mask");
    cv::Mat1b hit_mask(model_->GetHeight(), model_->GetWidth());
    int num_hit = 0;
    for (uint32_t i = 0; i < cloud_->points.size(); i++) {
      Eigen::Vector3d point(cloud_->points[i].x, cloud_->points[i].y,
                            cloud_->points[i].z);

      bool in_image = false;
      Eigen::Vector2d coords;
      if (!model_->ProjectPoint(point, coords, in_image)) {
        continue;
      } else if (!in_image) {
        continue;
      }
      uint16_t col = coords(0, 0);
      uint16_t row = coords(1, 0);
      hit_mask.at<uchar>(row, col) = 255;
      num_hit++;
      search_cloud_pt_to_orig_cloud_pt.push_back(i);
      search_cloud->push_back(pcl::PointXYZ(point[0], point[1], point[2]));
    }

    // create kdtree
    BEAM_DEBUG("creating kd search tree");
    beam::nanoflann::KdTree<pcl::PointXYZ> kdtree(search_cloud);

    // cast ray for every pixel in the hit mask
    int current = 1;
    for (int row = 0; row < image_->rows; row++) {
      for (int col = 0; col < image_->cols; col++) {
        if (hit_mask.at<uchar>(row, col) != 255) { continue; }

        beam::OutputPercentComplete(current, num_hit,
                                    "casting pixels from hit mask");
        Eigen::Vector3d ray(0, 0, 0);

        // get direction vector
        Eigen::Vector2i input_point(col, row);
        Eigen::Vector3d point;
        if (!model_->BackProject(input_point, point)) { return; }

        // while loop to ray trace
        uint16_t raypt = 0;
        while (raypt <= max_ray_extensions_) {
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
            int point_idx_orig = search_cloud_pt_to_orig_cloud_pt[point_idx[0]];
            behaviour(image_, cloud_, position, point_idx_orig);
            break;
          } else {
            raypt++;
            ray(0, 0) = ray(0, 0) + distance * point(0, 0);
            ray(1, 0) = ray(1, 0) + distance * point(1, 0);
            ray(2, 0) = ray(2, 0) + distance * point(2, 0);
          }
        }
        current++;
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
  int max_ray_extensions_;
};
} // namespace beam_cv
