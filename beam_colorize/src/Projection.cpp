#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include "beam_colorize/Projection.h"

namespace beam_colorize {

Projection::Projection() : Colorizer() {}

ProjectionMap Projection::CreateProjectionMap(
    const PointCloudCol::Ptr& cloud_in_camera_frame) const {
  ProjectionMap projection_map;
  for (uint32_t i = 0; i < cloud_in_camera_frame->points.size(); i++) {
    Eigen::Vector3d point(cloud_in_camera_frame->points[i].x,
                          cloud_in_camera_frame->points[i].y,
                          cloud_in_camera_frame->points[i].z);
    bool in_image = false;
    Eigen::Vector2d coords;
    if (!camera_model_->ProjectPoint(point, coords, in_image)) {
      continue;
    } else if (!in_image) {
      continue;
    }
    projection_map.Add(static_cast<uint64_t>(round(coords[0])),
                       static_cast<uint64_t>(round(coords[1])), i,
                       point.norm());
  }
  return projection_map;
}

ProjectionMap Projection::CreateProjectionMap(
    const DefectCloud::Ptr& cloud_in_camera_frame) const {
  ProjectionMap projection_map;
  for (uint32_t i = 0; i < cloud_in_camera_frame->points.size(); i++) {
    Eigen::Vector3d point(cloud_in_camera_frame->points[i].x,
                          cloud_in_camera_frame->points[i].y,
                          cloud_in_camera_frame->points[i].z);
    bool in_image = false;
    Eigen::Vector2d coords;
    if (!camera_model_->ProjectPoint(point, coords, in_image)) {
      continue;
    } else if (!in_image) {
      continue;
    }
    projection_map.Add(static_cast<uint64_t>(round(coords[0])),
                       static_cast<uint64_t>(round(coords[1])), i,
                       point.norm());
  }
  return projection_map;
}

} // namespace beam_colorize
