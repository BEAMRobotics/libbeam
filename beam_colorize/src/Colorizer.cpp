#include <beam_utils/log.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <beam_colorize/Colorizer.h>
#include <beam_colorize/Projection.h>
#include <beam_colorize/ProjectionOcclusionSafe.h>
#include <beam_colorize/RayTrace.h>
#include <beam_cv/OpenCVConversions.h>

namespace beam_colorize {

void ProjectionMap::Add(uint64_t u, uint64_t v, uint64_t point_id,
                        double depth) {
  auto v_iter = map_.find(v);

  // if v isn't found, add row
  if (v_iter == map_.end()) {
    UMapType u_map;
    u_map.emplace(u, ProjectedPointMeta(point_id, depth));
    map_.emplace(v, u_map);
    return;
  }

  auto u_iter = v_iter->second.find(u);
  // if u isn't found, add u and ID
  if (u_iter == v_iter->second.end()) {
    v_iter->second.emplace(u, ProjectedPointMeta(point_id, depth));
    return;
  }

  // else, check distance and keep point closest to camera
  if (u_iter->second.depth > depth) {
    u_iter->second = ProjectedPointMeta(point_id, depth);
  }
}

bool ProjectionMap::Get(uint64_t u, uint64_t v,
                        ProjectedPointMeta& point_meta) {
  auto v_iter = map_.find(v);
  if (v_iter == map_.end()) { return false; }
  auto u_iter = v_iter->second.find(u);
  if (u_iter == v_iter->second.end()) { return false; }
  point_meta = u_iter->second;
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

Colorizer::Colorizer() {
  image_distorted_ = true;
  image_initialized_ = false;
}

void Colorizer::SetImage(const cv::Mat& image_input) {
  image_ = std::make_shared<cv::Mat>(image_input);
  image_initialized_ = true;
}

void Colorizer::SetImage(const sensor_msgs::Image& image_input) {
  image_ = std::make_shared<cv::Mat>(
      beam_cv::OpenCVConversions::RosImgToMat(image_input));
  image_initialized_ = true;
}

void Colorizer::SetIntrinsics(
    std::shared_ptr<beam_calibration::CameraModel> intrinsics) {
  camera_model_distorted_ = intrinsics;
  camera_model_undistorted_ = camera_model_distorted_->GetRectifiedModel();
  camera_model_ = camera_model_distorted_;
}

void Colorizer::SetDistortion(const bool& image_distored) {
  image_distorted_ = image_distored;
  if (image_distored) {
    camera_model_ = camera_model_distorted_;
  } else {
    camera_model_ = camera_model_undistorted_;
  }
}

std::unique_ptr<Colorizer> Colorizer::Create(ColorizerType type) {
  if (type == ColorizerType::PROJECTION) {
    return std::make_unique<Projection>();
  } else if (type == ColorizerType::PROJECTION_OCCLUSION_SAFE) {
    return std::make_unique<ProjectionOcclusionSafe>();
  } else if (type == ColorizerType::RAY_TRACE) {
    return std::make_unique<RayTrace>();
  } else {
    BEAM_ERROR("Colorizer type not yet implemented in factory method");
    return nullptr;
  }
}

void Colorizer::ColorizePointCloud(
    DefectCloud::Ptr& cloud_in_camera_frame) const {
  if (!image_initialized_) {
    BEAM_CRITICAL("Colorizer not properly initialized: Image not initialized");
    throw std::runtime_error{"Colorizer not properly initialized."};
  }
  if (cloud_in_camera_frame->size() == 0) {
    BEAM_CRITICAL("Colorizer not properly initialized: empty cloud");
    throw std::runtime_error{"Colorizer not properly initialized."};
  }
  if (camera_model_ == nullptr) {
    BEAM_CRITICAL(
        "Colorizer not properly initialized: camera model not initialized");
    throw std::runtime_error{"Colorizer not properly initialized."};
  }

  ProjectionMap projection_map = CreateProjectionMap(cloud_in_camera_frame);
  int counter{0};
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
        cloud_in_camera_frame->points[u_iter->second.id].r = red;
        cloud_in_camera_frame->points[u_iter->second.id].g = green;
        cloud_in_camera_frame->points[u_iter->second.id].b = blue;
      }
    }
  }
  BEAM_INFO("Coloured {} of {} total points.", counter,
            cloud_in_camera_frame->points.size());
}

PointCloudCol::Ptr Colorizer::ColorizePointCloud(
    const PointCloud::Ptr& cloud_in_camera_frame) const {
  if (!image_initialized_ || camera_model_ == nullptr ||
      cloud_in_camera_frame->size() == 0) {
    BEAM_CRITICAL("Colorizer not properly initialized.");
    throw std::runtime_error{"Colorizer not properly initialized."};
  }

  auto cloud_colored = std::make_shared<PointCloudCol>();
  pcl::copyPointCloud(*cloud_in_camera_frame, *cloud_colored);

  ProjectionMap projection_map = CreateProjectionMap(cloud_colored);
  int counter{0};
  for (auto v_iter = projection_map.VBegin(); v_iter != projection_map.VEnd();
       v_iter++) {
    const auto& u_map = v_iter->second;
    for (auto u_iter = u_map.begin(); u_iter != u_map.end(); u_iter++) {
      cv::Vec3b colors = image_->at<cv::Vec3b>(v_iter->first, u_iter->first);
      uchar blue = colors.val[0];
      uchar green = colors.val[1];
      uchar red = colors.val[2];
      // ignore black colors, this happens at edges when images are undistorted
      if (red == 0 && green == 0 && blue == 0) {
        continue;
      } else {
        counter++;
        cloud_colored->points[u_iter->second.id].r = red;
        cloud_colored->points[u_iter->second.id].g = green;
        cloud_colored->points[u_iter->second.id].b = blue;
      }
    }
  }
  BEAM_INFO("Coloured {} of {} total points.", counter,
            cloud_in_camera_frame->points.size());
  return cloud_colored;
}

PointCloudCol::Ptr Colorizer::ColorizePointCloud(
    const PointCloudCol::Ptr& cloud_in_camera_frame) const {
  if (!image_initialized_ || camera_model_ == nullptr ||
      cloud_in_camera_frame->size() == 0) {
    BEAM_CRITICAL("Colorizer not properly initialized.");
    throw std::runtime_error{"Colorizer not properly initialized."};
  }

  auto cloud_colored = std::make_shared<PointCloudCol>();
  pcl::copyPointCloud(*cloud_in_camera_frame, *cloud_colored);

  ProjectionMap projection_map = CreateProjectionMap(cloud_colored);
  int counter{0};
  for (auto v_iter = projection_map.VBegin(); v_iter != projection_map.VEnd();
       v_iter++) {
    const auto& u_map = v_iter->second;
    for (auto u_iter = u_map.begin(); u_iter != u_map.end(); u_iter++) {
      cv::Vec3b colors = image_->at<cv::Vec3b>(v_iter->first, u_iter->first);
      uchar blue = colors.val[0];
      uchar green = colors.val[1];
      uchar red = colors.val[2];
      // ignore black colors, this happens at edges when images are undistorted
      if (red == 0 && green == 0 && blue == 0) {
        continue;
      } else {
        counter++;
        cloud_colored->points[u_iter->second.id].r = red;
        cloud_colored->points[u_iter->second.id].g = green;
        cloud_colored->points[u_iter->second.id].b = blue;
      }
    }
  }
  BEAM_INFO("Coloured {} of {} total points.", counter,
            cloud_in_camera_frame->points.size());
  return cloud_colored;
}

void Colorizer::ColorizeMask(DefectCloud::Ptr& cloud_in_camera_frame) const {
  if (!image_initialized_ || camera_model_ == nullptr ||
      cloud_in_camera_frame->size() == 0) {
    BEAM_CRITICAL("Colorizer not properly initialized.");
    throw std::runtime_error{"Colorizer not properly initialized."};
  }

  ProjectionMap projection_map = CreateProjectionMap(cloud_in_camera_frame);
  int counter{0};
  for (auto v_iter = projection_map.VBegin(); v_iter != projection_map.VEnd();
       v_iter++) {
    const auto& u_map = v_iter->second;
    for (auto u_iter = u_map.begin(); u_iter != u_map.end(); u_iter++) {
      uchar color_scale = image_->at<uchar>(v_iter->first, u_iter->first);
      if (color_scale == 0) {
        continue;
      } else if (color_scale == 1) {
        cloud_in_camera_frame->points[u_iter->second.id].crack = 1;
        counter++;
      } else if (color_scale == 2) {
        cloud_in_camera_frame->points[u_iter->second.id].delam = 1;
        counter++;
      } else if (color_scale == 3) {
        cloud_in_camera_frame->points[u_iter->second.id].corrosion = 1;
        counter++;
      } else if (color_scale == 4) {
        cloud_in_camera_frame->points[u_iter->second.id].spall = 1;
        counter++;
      } else {
        BEAM_ERROR("Mask value ({}) not suppored. Only 0-4 are supported",
                   color_scale);
      }
    }
  }

  BEAM_INFO("Coloured {} of {} total points using mask.", counter,
            cloud_in_camera_frame->points.size());
}

DefectCloud::Ptr Colorizer::ColorizeMask(
    const PointCloud::Ptr& cloud_in_camera_frame) const {
  if (!image_initialized_ || camera_model_ == nullptr ||
      cloud_in_camera_frame->size() == 0) {
    BEAM_CRITICAL("Colorizer not properly initialized.");
    throw std::runtime_error{"Colorizer not properly initialized."};
  }

  auto defect_cloud = std::make_shared<DefectCloud>();
  pcl::copyPointCloud(*cloud_in_camera_frame, *defect_cloud);
  ProjectionMap projection_map = CreateProjectionMap(defect_cloud);

  int counter{0};
  for (auto v_iter = projection_map.VBegin(); v_iter != projection_map.VEnd();
       v_iter++) {
    const auto& u_map = v_iter->second;
    for (auto u_iter = u_map.begin(); u_iter != u_map.end(); u_iter++) {
      uchar color_scale = image_->at<uchar>(v_iter->first, u_iter->first);
      if (color_scale == 0) {
        continue;
      } else if (color_scale == 1) {
        defect_cloud->points[u_iter->second.id].crack = 1;
        counter++;
      } else if (color_scale == 2) {
        defect_cloud->points[u_iter->second.id].delam = 1;
        counter++;
      } else if (color_scale == 3) {
        defect_cloud->points[u_iter->second.id].corrosion = 1;
        counter++;
      } else if (color_scale == 4) {
        defect_cloud->points[u_iter->second.id].spall = 1;
        counter++;
      } else {
        BEAM_ERROR("Mask value ({}) not suppored. Only 0-4 are supported",
                   color_scale);
      }
    }
  }
  BEAM_INFO("Coloured {} of {} total points using mask.", counter,
            cloud_in_camera_frame->points.size());
  return defect_cloud;
}

DefectCloud::Ptr Colorizer::ColorizeMask(
    const PointCloudCol::Ptr& cloud_in_camera_frame) const {
  if (!image_initialized_ || camera_model_ == nullptr ||
      cloud_in_camera_frame->size() == 0) {
    BEAM_CRITICAL("Colorizer not properly initialized.");
    throw std::runtime_error{"Colorizer not properly initialized."};
  }

  auto defect_cloud = std::make_shared<DefectCloud>();
  pcl::copyPointCloud(*cloud_in_camera_frame, *defect_cloud);
  ProjectionMap projection_map = CreateProjectionMap(defect_cloud);

  int counter{0};
  for (auto v_iter = projection_map.VBegin(); v_iter != projection_map.VEnd();
       v_iter++) {
    const auto& u_map = v_iter->second;
    for (auto u_iter = u_map.begin(); u_iter != u_map.end(); u_iter++) {
      int8_t color_scale = image_->at<int8_t>(v_iter->first, u_iter->first);
      if (color_scale == 0) {
        continue;
      } else if (color_scale == 1) {
        defect_cloud->points[u_iter->second.id].crack = 1;
        counter++;
      } else if (color_scale == 2) {
        defect_cloud->points[u_iter->second.id].delam = 1;
        counter++;
      } else if (color_scale == 3) {
        defect_cloud->points[u_iter->second.id].corrosion = 1;
        counter++;
      } else if (color_scale == 4) {
        defect_cloud->points[u_iter->second.id].spall = 1;
        counter++;
      } else {
        BEAM_ERROR("Mask value ({}) not suppored. Only 0-4 are supported",
                   color_scale);
      }
    }
  }
  BEAM_INFO("Coloured {} of {} total points using mask.", counter,
            cloud_in_camera_frame->points.size());
  return defect_cloud;
}

} // namespace beam_colorize
