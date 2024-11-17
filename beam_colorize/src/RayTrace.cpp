#include <beam_colorize/RayTrace.h>

#include <beam_cv/Raycast.h>

namespace beam_colorize {

RayTrace::RayTrace() : Colorizer() {}

ProjectionMap RayTrace::CreateProjectionMap(
    const PointCloudCol::Ptr& cloud_in_camera_frame) const {
  beam_cv::Raycast<pcl::PointXYZRGB> caster(cloud_in_camera_frame,
                                            camera_model_, image_);
  // perform ray casting of cloud to colorize with image
  ProjectionMap projection_map;
  caster.Execute(hit_threshold_,
                 [&](std::shared_ptr<cv::Mat>& image,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                     const int* position, int index) -> void {
                   Pixel& pixel = image->at<Pixel>(position[1], position[0]);
                   double depth = beam::CalculatePointNorm<pcl::PointXYZRGB>(
                       cloud->points[index]);
                   projection_map.Add(position[0], position[1], index, depth);
                 });
  return projection_map;
}

ProjectionMap RayTrace::CreateProjectionMap(
    const DefectCloud::Ptr& cloud_in_camera_frame) const {
  beam_cv::Raycast<beam_containers::PointBridge> caster(cloud_in_camera_frame,
                                                        camera_model_, image_);
  // perform ray casting of cloud to colorize with image
  ProjectionMap projection_map;
  caster.Execute(hit_threshold_,
                 [&](std::shared_ptr<cv::Mat>& image,
                     pcl::PointCloud<beam_containers::PointBridge>::Ptr& cloud,
                     const int* position, int index) -> void {
                   Pixel& pixel = image->at<Pixel>(position[1], position[0]);
                   double depth =
                       beam::CalculatePointNorm<beam_containers::PointBridge>(
                           cloud->points[index]);
                   projection_map.Add(position[1], position[0], index, depth);
                 });
  return projection_map;
}

} // namespace beam_colorize
