#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include "beam_colorize/ProjectionOcclusionSafe.h"

namespace beam_colorize {

void ProjectionMap::Add(uint64_t u, uint64_t v, uint64_t point_id){
  auto v_iter = map_.find(v);

  // if v isn't found, add row
  if(v_iter == map_.end()){
    std::unordered_map<uint64_t, std::vector<uint64_t>> map2;
    map2.emplace(u, std::vector<uint64_t>{point_id});
    map_.emplace(v, map2);
    return;
  } 

  u_iter = v_iter->second.find(u);
  // if u isn't found, add new vector of ids
  if(u_iter == v_iter->second.end()){
    v_iter->second.emplace(u, std::vector<uint64_t>{point_id});
    return;
  }

  u_iter->second.push_back(point_id);
}

const std::vector<uint64_t>& ProjectionMap::Get(uint64_t u, uint64_t v){
  auto v_iter = map_.find(v);

  if(v_iter == map_.end()){
    return std::vector<uint64_t>{};
  } 

  u_iter = v_iter->second.find(u);

  if(u_iter == v_iter->second.end()){
    return std::vector<uint64_t>{};
  }

  return u_iter->second;
}

ProjectionOcclusionSafe::ProjectionOcclusionSafe() : Colorizer() {}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    ProjectionOcclusionSafe::ColorizePointCloud() const {
  auto cloud_colored = std::make_shared<PointCloudCol>();

  if (!image_initialized_ || camera_model_ == nullptr ||
      cloud_in_camera_frame_->size() == 0) {
    BEAM_CRITICAL("Colorizer not properly initialized, image initialized: {}, "
                  "camera model initialized: {}",
                  image_initialized_, camera_model_ != nullptr);
    throw std::runtime_error{"Colorizer not properly initialized."};
  }


  // TODO

  return cloud_colored;
}

pcl::PointCloud<beam_containers::PointBridge>::Ptr
    ProjectionOcclusionSafe::ColorizeMask() const {
  auto defect_cloud = std::make_shared<beam_containers::PointBridge>();

  // TODO

  return defect_cloud;
}

} // namespace beam_colorize
