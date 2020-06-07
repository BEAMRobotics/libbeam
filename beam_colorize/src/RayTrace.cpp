#include <pcl/kdtree/kdtree_flann.h>

#include "beam_colorize/RayTrace.h"
#include "beam_cv/RayCast.h"

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
    uint16_t u = std::round(coords.value()(0, 0)),
             v = std::round(coords.value()(1, 0));
    uint16_t vmax = image->rows;
    uint16_t umax = image->cols;
    if (u > 0 && v > 0 && v < vmax && u < umax && point(2, 0) > 0) {
      pcl::PointXYZRGB new_point;
      new_point.x = point(0, 0);
      new_point.y = point(1, 0);
      new_point.z = point(2, 0);
      cloud->points.push_back(new_point);
      indices.push_back(i);
    }
  }
  return std::make_tuple(cloud, indices);
}

pcl::PointCloud<beam_containers::PointBridge>::Ptr
    RayTrace::ColorizeMask() const {
  return pcl::PointCloud<beam_containers::PointBridge>::Ptr();
}

} // namespace beam_colorize
