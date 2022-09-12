#include "beam_colorize/ProjectionOcclusionSafe.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <beam_utils/pointclouds.h>

namespace beam_colorize {

void ProjectionMap::Add(uint64_t u, uint64_t v, uint64_t point_id) {
  auto v_iter = map_.find(v);

  // if v isn't found, add row
  if (v_iter == map_.end()) {
    std::unordered_map<uint64_t, uint64_t> map2;
    map2.emplace(u, point_id);
    map_.emplace(v, map2);
    return;
  }

  auto u_iter = v_iter->second.find(u);
  // if u isn't found, add u and ID
  if (u_iter == v_iter->second.end()) {
    v_iter->second.emplace(u, point_id);
    return;
  }

  // else, check distance and keep point closest to camera
  if (beam::CalculatePointNorm(cloud_->points.at(u_iter->second)) >
      beam::CalculatePointNorm(cloud_->points.at(point_id))) {
    u_iter->second = point_id;
  }
}

bool ProjectionMap::Get(uint64_t u, uint64_t v, uint64_t& id) {
  auto v_iter = map_.find(v);
  if (v_iter == map_.end()) { return false; }
  auto u_iter = v_iter->second.find(u);
  if (u_iter == v_iter->second.end()) { return false; }
  id = u_iter->second;
}

void ProjectionMap::Erase(uint64_t u, uint64_t v) {
  auto v_iter = map_.find(v);
  if (v_iter == map_.end()) { return; }
  auto u_iter = v_iter->second.find(u);
  if (u_iter == v_iter->second.end()) { return; }
  v_iter->second.erase(u);
}

ProjectionOcclusionSafe::ProjectionOcclusionSafe() : Colorizer() {}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    ProjectionOcclusionSafe::ColorizePointCloud() const {
  ProjectionMap map(cloud_in_camera_frame_);

  auto cloud_colored = std::make_shared<PointCloudCol>();

  if (!image_initialized_ || camera_model_ == nullptr ||
      cloud_in_camera_frame_->size() == 0) {
    BEAM_CRITICAL("Colorizer not properly initialized, image initialized: {}, "
                  "camera model initialized: {}",
                  image_initialized_, camera_model_ != nullptr);
    throw std::runtime_error{"Colorizer not properly initialized."};
  }

  for (uint64_t i = 0; i < cloud_in_camera_frame_->points.size(); i++) {
    Eigen::Vector3d point(cloud_in_camera_frame_->points[i].x,
                          cloud_in_camera_frame_->points[i].y,
                          cloud_in_camera_frame_->points[i].z);
    bool in_image = false;
    Eigen::Vector2d coords;
    if (!camera_model_->ProjectPoint(point, coords, in_image)) {
      continue;
    } else if (!in_image) {
      continue;
    }

    map.Add(coords[0], coords[1], i);
  }

  RemoveOccludedPointsFromMap(map);

  return cloud_colored;
}

pcl::PointCloud<beam_containers::PointBridge>::Ptr
    ProjectionOcclusionSafe::ColorizeMask() const {
  auto defect_cloud =
      std::make_shared<pcl::PointCloud<beam_containers::PointBridge>>();

  // TODO

  return defect_cloud;
}

void ProjectionOcclusionSafe::RemoveOccludedPointsFromMap(ProjectionMap& map) {
  uint64_t u_max = image_->cols;
  uint64_t v_max = image_->rows;
  for (uint64_t v = 0; v < v_max; v += window_stride_) {
    for (uint64_t u = 0; u < u_max; u += window_stride_) {
      RemoveOccludedPointsFromWindow(map, u, v);
    }
  }
}

void ProjectionOcclusionSafe::RemoveOccludedPointsFromWindow(ProjectionMap& map,
                                                             uint64_t u_start,
                                                             uint64_t v_start) {
  int num_points{0};
  float sum_distances{0};
  for (uint64_t v = v_start; v < v_start + window_size_; v++) {
    for (uint64_t u = u_start; u < u_start + window_size_; u++) {
      // get point id projected to this pixel
      uint64_t id;
      if (!map.Get(u, v, id)) { continue; }

      // check id's distance against current mean, and remove if larger than
      // threshold, otherwise update sum and number of points in this
      // neighborhood
      float point_distance =
          beam::CalculatePointNorm(cloud_in_camera_frame_->points.at(id));
      if (num_points == 0) {
        num_points++;
        sum_distances = point_distance;
      } else if (std::abs(point_distance - sum_distances / num_points) <
                 depth_segmentation_threshold_m_) {
        num_points++;
        sum_distances += point_distance;
      } else {
        map.Erase(u, v);
      }
    }
  }
}

} // namespace beam_colorize
