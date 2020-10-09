#include <beam_colorize/RayTrace.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <beam_cv/RayCast.h>

namespace beam_colorize {

void HitBehaviour(std::shared_ptr<cv::Mat> image,
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                  const int* position, int index) {
  Pixel& pixel = image->at<Pixel>(position[0], position[1]);
  cloud->points[index].r = pixel.z;
  cloud->points[index].g = pixel.y;
  cloud->points[index].b = pixel.x;
}

RayTrace::RayTrace() : Colorizer() {}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RayTrace::ColorizePointCloud() const {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

  pcl::copyPointCloud(*input_point_cloud_, *cloud_colored);

  if (!image_initialized_ || !point_cloud_initialized_ ||
      !intrinsics_initialized_ || input_point_cloud_->size() == 0) {
    throw std::runtime_error{"Colorizer not properly initialized."};
    BEAM_CRITICAL("Colorizer not properly initialized.");
    return cloud_colored;
  }
  // create image mask where white pixels = projection hit
  cv::Mat hit_mask =
      beam_cv::CreateHitMask(dilation_, intrinsics_, input_point_cloud_);
  // This lambda performs ray tracing in parallel on each pixel in the image
  beam_cv::RayCast(image_, cloud_colored, hit_mask, hit_threshold_, intrinsics_,
                   HitBehaviour);

  return cloud_colored;
}

std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, std::vector<int>>
    RayTrace::ReduceCloud(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
        std::shared_ptr<cv::Mat> image,
        std::shared_ptr<beam_calibration::CameraModel> intrinsics) const {
  // cloud to search on
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<int> indices;
  beam::Vec3 point;

  for (uint32_t i = 0; i < input->points.size(); i++) {
    point << input->points[i].x, input->points[i].y, input->points[i].z;
    opt<Eigen::Vector2i> coords = intrinsics->ProjectPoint(point);
    if (!coords.has_value()) {
      BEAM_WARN("Cannot project point.");
      continue;
    }
    uint16_t u = coords.value()(0, 0), v = coords.value()(1, 0);
    pcl::PointXYZRGB new_point;
    new_point.x = point(0, 0);
    new_point.y = point(1, 0);
    new_point.z = point(2, 0);
    cloud->points.push_back(new_point);
    indices.push_back(i);
  }
  return std::make_tuple(cloud, indices);
}

pcl::PointCloud<beam_containers::PointBridge>::Ptr
    RayTrace::ColorizeMask() const {
  cv::Mat hit_mask =
      beam_cv::CreateHitMask(dilation_, intrinsics_, input_point_cloud_);
  // create and fill point bridge cloud
  pcl::PointCloud<beam_containers::PointBridge>::Ptr return_cloud;
  pcl::copyPointCloud(*input_point_cloud_, *return_cloud);

  pcl::PointXYZ origin(0, 0, 0);
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud(input_point_cloud_);
  int counter = 0;
  // ray trace every valid pixel
  image_->forEach<uchar>([&](uchar& pixel, const int* position) -> void {
    (void)pixel;
    int row = position[0], col = position[1];
    if (hit_mask.at<cv::Vec3b>(row, col).val[0] == 255) {
      Eigen::Vector3d ray(0, 0, 0);
      // get direction vector
      Eigen::Vector2i input_point(col, row);
      opt<Eigen::Vector3d> point = intrinsics_->BackProject(input_point);
      if (!point.has_value()) { return; }
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
        // if the point is within 1cm then label with specific defect
        if (distance < hit_threshold_) {
          uchar color_scale = image_->at<uchar>(row, col);
          if (color_scale == 0) {
            continue;
          } else if (color_scale == 1) {
            return_cloud->points[point_idx[0]].crack = 1;
            counter++;
          } else if (color_scale == 2) {
            return_cloud->points[point_idx[0]].delam = 1;
            counter++;
          } else if (color_scale == 3) {
            return_cloud->points[point_idx[0]].corrosion = 1;
            counter++;
          } else if (color_scale == 4) {
            return_cloud->points[point_idx[0]].spall = 1;
            counter++;
          }
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
  BEAM_INFO("Coloured {} of {} total points.", counter,
            input_point_cloud_->points.size());
  return return_cloud;
}

} // namespace beam_colorize
