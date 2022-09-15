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
  RemoveOccludedPointsFromMap(projection_map);

  // color cloud
  auto cloud_colored = std::make_shared<PointCloudCol>();
  pcl::copyPointCloud(*cloud_in_camera_frame_, *cloud_colored);
  int counter = 0;
  for (auto v_iter = projection_map.VBegin(); v_iter != projection_map.VEnd();
       v_iter++) {
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

void ProjectionOcclusionSafe::RemoveOccludedPointsFromMap(
    ProjectionMap& projection_map) const {
  uint64_t u_max = image_->cols;
  uint64_t v_max = image_->rows;
  for (uint64_t v = 0; v < v_max; v += window_stride_) {
    for (uint64_t u = 0; u < u_max; u += window_stride_) {
      RemoveOccludedPointsFromWindow(projection_map, u, v);
    }
  }
}

void ProjectionOcclusionSafe::RemoveOccludedPointsFromWindow(
    ProjectionMap& projection_map, uint64_t u_start, uint64_t v_start) const {
  // map: distance -> [u, v]
  std::map<float, std::vector<uint64_t>> points;
  for (uint64_t v = v_start; v < v_start + window_size_; v++) {
    for (uint64_t u = u_start; u < u_start + window_size_; u++) {
      // get point id projected to this pixel
      uint64_t id;
      if (!projection_map.Get(u, v, id)) { continue; }

      // add point to sorted map
      float d = beam::CalculatePointNorm(cloud_in_camera_frame_->points.at(id));
      points.emplace(d, std::vector<uint64_t>{u, v});
    }
  }

  if (points.size() < 2) { return; }

  // iterate through map sorted by distance and remove all points where the gap
  // is larger than the threshold
  for (auto curr_iter = std::next(points.begin()); curr_iter != points.end();
       curr_iter++) {
    auto prev_iter = std::prev(curr_iter);
    if (curr_iter->first - prev_iter->first > depth_seg_thresh_m_) {
      // remove current and all remaining points
      for (auto iter = curr_iter; iter != points.end(); iter++) {
        projection_map.Erase(iter->second[0], iter->second[1]);
      }
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

void ProjectionOcclusionSafe::SetDepthThreshold(uint8_t depth_seg_thresh_m) {
  depth_seg_thresh_m_ = depth_seg_thresh_m;
}

} // namespace beam_colorize
