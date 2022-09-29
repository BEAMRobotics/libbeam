#include "beam_colorize/ProjectionOcclusionSafe.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <beam_utils/pointclouds.h>

namespace beam_colorize {

ProjectionMap::ProjectionMap(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in_camera_frame)
    : cloud_(cloud_in_camera_frame) {}

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
  return true;
}

void ProjectionMap::Erase(uint64_t u, uint64_t v) {
  auto v_iter = map_.find(v);
  if (v_iter == map_.end()) { return; }
  auto u_iter = v_iter->second.find(u);
  if (u_iter == v_iter->second.end()) { return; }
  v_iter->second.erase(u);
}

int ProjectionMap::Size() {
  int counter = 0;
  for (auto v_iter = VBegin(); v_iter != VEnd(); v_iter++) {
    for (auto u_iter = v_iter->second.begin(); u_iter != v_iter->second.end();
         u_iter++) {
      counter++;
    }
  }
  return counter;
}

std::unordered_map<uint64_t, UMapType>::iterator ProjectionMap::VBegin() {
  return map_.begin();
}

std::unordered_map<uint64_t, UMapType>::iterator ProjectionMap::VEnd() {
  return map_.end();
}

ProjectionOcclusionSafe::ProjectionOcclusionSafe() : Colorizer() {}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    ProjectionOcclusionSafe::ColorizePointCloud() const {
  // check class is initialized correctly
  if (!image_initialized_ || camera_model_ == nullptr ||
      cloud_in_camera_frame_->size() == 0) {
    BEAM_CRITICAL("Colorizer not properly initialized, image initialized: {}, "
                  "camera model initialized: {}",
                  image_initialized_, camera_model_ != nullptr);
    throw std::runtime_error{"Colorizer not properly initialized."};
  }

  // build projection map
  ProjectionMap projection_map(cloud_in_camera_frame_);
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
    projection_map.Add(coords[0], coords[1], i);
  }
  int valid_projections = projection_map.Size();

  ProjectionMap projection_map_final =
      RemoveOccludedPointsFromMap(projection_map);
  BEAM_INFO("Colorizing {} points from {} total valid projections",
            projection_map_final.Size(), valid_projections);

  // color cloud
  auto cloud_colored = std::make_shared<PointCloudCol>();
  pcl::copyPointCloud(*cloud_in_camera_frame_, *cloud_colored);
  int counter = 0;
  for (auto v_iter = projection_map_final.VBegin();
       v_iter != projection_map_final.VEnd(); v_iter++) {
    const auto& u_map = v_iter->second;
    for (auto u_iter = u_map.begin(); u_iter != u_map.end(); u_iter++) {
      cv::Vec3b colors = image_->at<cv::Vec3b>(v_iter->first, u_iter->first);
      uchar blue = colors.val[0];
      uchar green = colors.val[1];
      uchar red = colors.val[2];
      // ignore black colors, this happens at edges when images are undistored
      if (red == 0 && green == 0 && blue == 0) {
        continue;
      } else {
        counter++;
        cloud_colored->points[u_iter->second].r = red;
        cloud_colored->points[u_iter->second].g = green;
        cloud_colored->points[u_iter->second].b = blue;
      }
    }
  }
  BEAM_INFO("Coloured {} of {} total points.", counter,
            cloud_in_camera_frame_->points.size());

  return cloud_colored;
}

pcl::PointCloud<beam_containers::PointBridge>::Ptr
    ProjectionOcclusionSafe::ColorizeMask() const {
  auto defect_cloud =
      std::make_shared<pcl::PointCloud<beam_containers::PointBridge>>();

  // TODO

  return defect_cloud;
}

ProjectionMap ProjectionOcclusionSafe::RemoveOccludedPointsFromMap(
    ProjectionMap& projection_map_orig) const {
  uint64_t u_max = image_->cols;
  uint64_t v_max = image_->rows;
  ProjectionMap projection_map_to_keep(cloud_in_camera_frame_);
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
      uint64_t id;
      if (!projection_map.Get(u, v, id)) { continue; }

      // add point to sorted map
      auto p = cloud_in_camera_frame_->points.at(id);
      float d = beam::CalculatePointNorm(cloud_in_camera_frame_->points.at(id));
      points.emplace(d, ProjectedPoint{.u = u, .v = v, .id = id});
    }
  }

  if (points.empty()) { return; }

  // add the first point as we know it'll have the lowest depth
  projection_map_to_keep.Add(points.begin()->second.u, points.begin()->second.v,
                             points.begin()->second.id);

  // iterate through map sorted by distance and colorize all points up until
  // there's a jump in distance more than the threshold. Keep others in map to
  // be revisited in another window. 
  for (auto curr_iter = std::next(points.begin()); curr_iter != points.end();
       curr_iter++) {
    auto prev_iter = std::prev(curr_iter);
    if (curr_iter->first - prev_iter->first < depth_seg_thresh_m_) {
      projection_map.Erase(curr_iter->second.u, curr_iter->second.v);
      projection_map_to_keep.Add(curr_iter->second.u, curr_iter->second.v,
                                 curr_iter->second.id);
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

void ProjectionOcclusionSafe::SetDepthThreshold(double depth_seg_thresh_m) {
  depth_seg_thresh_m_ = depth_seg_thresh_m;
}

} // namespace beam_colorize
