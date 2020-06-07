#include <beam_cv/RayCast.h>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <beam_utils/math.hpp>

using namespace cv;
namespace beam_cv {

void RayCast(std::shared_ptr<cv::Mat> image,
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
    (void)pixel;
    int v = position[0], u = position[1];
    if (hit_mask.at<cv::Vec3b>(v, u).val[0] == 255) {
      Eigen::Vector3d ray(0, 0, 0);
      // get direction vector
      Eigen::Vector2i input_point(u, v);
      opt<Eigen::Vector3d> point = model->BackProject(input_point);
      if (!point.has_value()) {
        BEAM_WARN("Unable to back project pixel.");
        return;
      }
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
          ray(0, 0) = ray(0, 0) + distance * point.value()(0, 0);
          ray(1, 0) = ray(1, 0) + distance * point.value()(1, 0);
          ray(2, 0) = ray(2, 0) + distance * point.value()(2, 0);
        }
      }
    }
  });
}

void RayCast(std::shared_ptr<cv::Mat> image,
             pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cv::Mat hit_mask,
             float threshold,
             std::shared_ptr<beam_calibration::CameraModel> model,
             void (*f)(std::shared_ptr<cv::Mat> image_i,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_i,
                       const int* position, int index)) {
  pcl::PointXYZ origin(0, 0, 0);
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud(cloud);
  /// Compute depth image with point cloud
  image->forEach<float>([&](float& pixel, const int* position) -> void {
    (void)pixel;
    int v = position[0], u = position[1];
    if (hit_mask.at<cv::Vec3b>(v, u).val[0] == 255) {
      Eigen::Vector3d ray(0, 0, 0);
      // get direction vector
      Eigen::Vector2i input_point(u, v);
      opt<Eigen::Vector3d> point = model->BackProject(input_point);
      if (!point.has_value()) {
        BEAM_WARN("Unable to back project pixel.");
        return;
      }
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
          ray(0, 0) = ray(0, 0) + distance * point.value()(0, 0);
          ray(1, 0) = ray(1, 0) + distance * point.value()(1, 0);
          ray(2, 0) = ray(2, 0) + distance * point.value()(2, 0);
        }
      }
    }
  });
}

cv::Mat CreateHitMask(int mask_size,
                      std::shared_ptr<beam_calibration::CameraModel> model,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  // create image mask where white pixels = projection hit
  cv::Size image_s(model->GetHeight(), model->GetWidth());
  cv::Mat tmp = cv::Mat::zeros(image_s, CV_8UC3);
  cv::Mat hit_mask;
  for (uint32_t i = 0; i < cloud->points.size(); i++) {
    Eigen::Vector3d point;
    point << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z;
    opt<Eigen::Vector2i> coords = model->ProjectPoint(point);
    if (!coords.has_value()) {
      BEAM_WARN("Unable to back project pixel.");
      continue;
    }
    uint16_t u = std::round(coords.value()(0, 0)),
             v = std::round(coords.value()(1, 0));
    if (u > 0 && v > 0 && v < model->GetHeight() && u < model->GetWidth()) {
      tmp.at<cv::Vec3b>(v, u).val[0] = 255;
    }
  }
  cv::dilate(tmp, hit_mask, cv::Mat(mask_size, mask_size, CV_8UC1),
             cv::Point(-1, -1), 1, 1, 1);
  return hit_mask;
}

cv::Mat CreateHitMask(int mask_size,
                      std::shared_ptr<beam_calibration::CameraModel> model,
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  // create image mask where white pixels = projection hit
  cv::Size image_s(model->GetHeight(), model->GetWidth());
  cv::Mat tmp = cv::Mat::zeros(image_s, CV_8UC3);
  cv::Mat hit_mask;
  for (uint32_t i = 0; i < cloud->points.size(); i++) {
    Eigen::Vector3d point;
    point << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z;
    opt<Eigen::Vector2i> coords = model->ProjectPoint(point);
    if (!coords.has_value()) {
      BEAM_WARN("Unable to back project pixel.");
      continue;
    }
    uint16_t u = std::round(coords.value()(0, 0)),
             v = std::round(coords.value()(1, 0));
    if (u > 0 && v > 0 && v < model->GetWidth() && u < model->GetHeight()) {
      tmp.at<cv::Vec3b>(u, v).val[0] = 255;
    }
  }
  cv::dilate(tmp, hit_mask, cv::Mat(mask_size, mask_size, CV_8UC1),
             cv::Point(-1, -1), 1, 1, 1);
  return hit_mask;
}

} // namespace beam_cv
