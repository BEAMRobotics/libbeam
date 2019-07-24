#include "beam_cv/RayCast.h"

using namespace cv;
namespace beam_cv {

void RayCastXYZ(std::shared_ptr<cv::Mat> image,
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cv::Mat hit_mask,
                float threshold,
                std::shared_ptr<beam_calibration::CameraModel> model,
                void (*f)(std::shared_ptr<cv::Mat> image_i,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_i,
                          const int* position, int index)) {
  /// create image with 3 channels for coordinates
  pcl::PointXYZ origin(0, 0, 0);
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);
  /// Compute depth image with point cloud
  image->forEach<float>([&](float& pixel, const int* position) -> void {
    int v = position[0], u = position[1];
    if (hit_mask.at<cv::Vec3b>(v, u).val[0] == 255) {
      beam::Vec3 ray(0, 0, 0);
      // get direction vector
      beam::Vec2 input_point(u, v);
      beam::Vec3 point = model->BackProject(input_point);
      // while loop to ray trace
      uint16_t raypt = 0;
      while (true) {
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
        // if the point is within 1cm then color it appropriately
        if (distance < threshold) {
          (*f)(image, cloud, position, point_idx[0]);
          break;
        } else if (raypt >= 20) {
          break;
        } else {
          raypt++;
          ray(0, 0) = ray(0, 0) + distance * point(0, 0);
          ray(1, 0) = ray(1, 0) + distance * point(1, 0);
          ray(2, 0) = ray(2, 0) + distance * point(2, 0);
        }
      }
    }
  });
}

void RayCastXYZRGB(std::shared_ptr<cv::Mat> image,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                   cv::Mat hit_mask, float threshold,
                   std::shared_ptr<beam_calibration::CameraModel> model,
                   void (*f)(std::shared_ptr<cv::Mat> image_i,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_i,
                             const int* position, int index)) {
  /// create image with 3 channels for coordinates
  pcl::PointXYZ origin(0, 0, 0);
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud(cloud);
  /// Compute depth image with point cloud
  image->forEach<float>([&](float& pixel, const int* position) -> void {
    int v = position[0], u = position[1];
    if (hit_mask.at<cv::Vec3b>(v, u).val[0] == 255) {
      beam::Vec3 ray(0, 0, 0);
      // get direction vector
      beam::Vec2 input_point(u, v);
      beam::Vec3 point = model->BackProject(input_point);
      // while loop to ray trace
      uint16_t raypt = 0;
      while (true) {
        // get point at end of ray
        pcl::PointXYZRGB search_point;
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
          (*f)(image, cloud, position, point_idx[0]);
          break;
        } else if (raypt >= 20) {
          break;
        } else {
          raypt++;
          ray(0, 0) = ray(0, 0) + distance * point(0, 0);
          ray(1, 0) = ray(1, 0) + distance * point(1, 0);
          ray(2, 0) = ray(2, 0) + distance * point(2, 0);
        }
      }
    }
  });
}
} // namespace beam_cv