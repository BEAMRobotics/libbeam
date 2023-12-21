#include "beam_colorize/ProjectionOcclusionSafe.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <beam_utils/pointclouds.h>

namespace beam_colorize {

ProjectionOcclusionSafe::ProjectionOcclusionSafe() : Colorizer() {}

ProjectionMap ProjectionOcclusionSafe::CreateProjectionMap(
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
  ProjectionMap projection_map_final =
      RemoveOccludedPointsFromMap(projection_map);
  return projection_map_final;
}

ProjectionMap ProjectionOcclusionSafe::CreateProjectionMap(
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
  ProjectionMap projection_map_final =
      RemoveOccludedPointsFromMap(projection_map);
  return projection_map_final;
}

ProjectionMap ProjectionOcclusionSafe::RemoveOccludedPointsFromMap(
    ProjectionMap& projection_map_orig) const {
  uint64_t u_max = image_->cols;
  uint64_t v_max = image_->rows;
  ProjectionMap projection_map_to_keep;
  for (uint64_t v = 0; v < v_max; v += window_stride_) {
    for (uint64_t u = 0; u < u_max; u += window_stride_) {
      CheckOcclusionsInWindow(projection_map_to_keep, projection_map_orig, u,
                              v);
    }
  }
  return projection_map_to_keep;
}

void ProjectionOcclusionSafe::CheckOcclusionsInWindow(
    ProjectionMap& projection_map_to_keep, ProjectionMap& projection_map,
    uint64_t u_start, uint64_t v_start) const {
  // map: distance -> {u, v, id}
  std::map<float, ProjectedPoint> points;
  for (uint64_t v = v_start; v < v_start + window_size_; v++) {
    for (uint64_t u = u_start; u < u_start + window_size_; u++) {
      // get point id projected to this pixel. If none exists, then skip pixel
      ProjectedPointMeta point_meta(0, 0);
      if (!projection_map.Get(u, v, point_meta)) { continue; }

      // add point to sorted map
      points.emplace(point_meta.depth,
                     ProjectedPoint{.u = u, .v = v, .id = point_meta.id});
    }
  }

  if (points.size() < 2) { return; }

  // calculate average change in depth
  double delta_d_sum = 0;
  for (auto it = std::next(points.begin()); it != points.end(); it++) {
    double delta_d = it->first - std::prev(it)->first;
    delta_d_sum += delta_d;
  }
  double delta_d_avg = delta_d_sum / (points.size() - 1);
  std::cout << "delta_d_avg: " << delta_d_avg << "\n";
  // add the first point as we know it'll have the lowest depth
  projection_map_to_keep.Add(points.begin()->second.u, points.begin()->second.v,
                             points.begin()->second.id, points.begin()->first);

  // iterate through map sorted by distance and colorize all points up until
  // there's a jump in distance more than the threshold. Keep others in map to
  // be revisited in another window.
  for (auto curr_iter = std::next(points.begin()); curr_iter != points.end();
       curr_iter++) {
    auto prev_iter = std::prev(curr_iter);
    if (curr_iter->first - prev_iter->first <
        delta_depth_avg_multiplier_ * delta_d_avg) {
      projection_map.Erase(curr_iter->second.u, curr_iter->second.v);
      projection_map_to_keep.Add(curr_iter->second.u, curr_iter->second.v,
                                 curr_iter->second.id, curr_iter->second.depth);
    } else {
      break;
    }
  }
}

void ProjectionOcclusionSafe::SetWindowSize(uint8_t window_size) {
  window_size_ = window_size;
}

void ProjectionOcclusionSafe::SetWindowStride(uint8_t window_stride) {
  window_stride_ = window_stride;
}

} // namespace beam_colorize
