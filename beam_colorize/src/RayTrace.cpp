#include <beam_colorize/RayTrace.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <beam_cv/Raycast.h>

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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    RayTrace::ColorizePointCloud(bool return_in_cam_frame) const {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored =
      std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

  pcl::copyPointCloud(*cloud_in_camera_frame_, *cloud_colored);

  if (!image_initialized_ || camera_model_ == nullptr ||
      cloud_in_camera_frame_->size() == 0) {
    throw std::runtime_error{"Colorizer not properly initialized."};
    BEAM_CRITICAL("Colorizer not properly initialized.");
    return cloud_colored;
  }
  beam_cv::Raycast<pcl::PointXYZRGB> caster(cloud_colored, camera_model_,
                                            image_);
  // perform ray casting of cloud to colorize with image
  caster.Execute(hit_threshold_,
                 [&](std::shared_ptr<cv::Mat>& image,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                     const int* position, int index) -> void {
                   Pixel& pixel = image->at<Pixel>(position[0], position[1]);
                   cloud->points[index].r = pixel.z;
                   cloud->points[index].g = pixel.y;
                   cloud->points[index].b = pixel.x;
                 });
  if (return_in_cam_frame) {
    return cloud_colored;
  } else {
    return GetCloudInLidarFrame(cloud_colored);
  }
}

std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, std::vector<int>>
    RayTrace::ReduceCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input) const {
  // cloud to search on
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<int> indices;
  Eigen::Vector3d point;

  for (uint32_t i = 0; i < input->points.size(); i++) {
    point << input->points[i].x, input->points[i].y, input->points[i].z;
    bool in_image = false;
    Eigen::Vector2d coords;
    if (!camera_model_->ProjectPoint(point, coords, in_image)) {
      BEAM_WARN("Cannot project point.");
      continue;
    } else if (!in_image) {
      BEAM_WARN("Cannot project point.");
      continue;
    }
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
    RayTrace::ColorizeMask(bool return_in_cam_frame) const {
  pcl::PointCloud<beam_containers::PointBridge>::Ptr return_cloud;
  pcl::copyPointCloud(*cloud_in_camera_frame_, *return_cloud);

  beam_cv::Raycast<beam_containers::PointBridge> caster(return_cloud,
                                                        camera_model_, image_);
  // perform ray casting of cloud to color with mask
  int counter = 0;
  caster.Execute(hit_threshold_,
                 [&](std::shared_ptr<cv::Mat>& image,
                     pcl::PointCloud<beam_containers::PointBridge>::Ptr& cloud,
                     const int* position, int index) -> void {
                   uchar color_scale =
                       image->at<uchar>(position[0], position[1]);
                   switch (color_scale) {
                     case 1:
                       cloud->points[index].crack = 1;
                       counter++;
                       break;
                     case 2:
                       cloud->points[index].delam = 1;
                       counter++;
                       break;
                     case 3:
                       cloud->points[index].corrosion = 1;
                       counter++;
                       break;
                     case 4:
                       cloud->points[index].spall = 1;
                       counter++;
                       break;
                   }
                 });
  BEAM_INFO("Coloured {} of {} total points.", counter,
            cloud_in_camera_frame_->points.size());
  if (return_in_cam_frame) {
    return return_cloud;
  } else {
    return GetCloudInLidarFrame(return_cloud);
  }
}

} // namespace beam_colorize
