#include <beam_depth/DepthMap.h>

#include <pcl/io/pcd_io.h>

#include <beam_cv/RayCast.h>
#include <beam_cv/Utils.h>
#include <beam_depth/Utils.h>
#include <beam_utils/math.hpp>

// TODO: Why is this outside the namespace?
void HitBehaviour(std::shared_ptr<cv::Mat> image,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                  const int* position, int index) {
  pcl::PointXYZ origin(0, 0, 0);
  image->at<float>(position[0], position[1]) =
      beam::distance(cloud->points[index], origin);
}

namespace beam_depth {

DepthMap::DepthMap(std::shared_ptr<beam_calibration::CameraModel> model,
                   const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input) {
  this->SetCloud(cloud_input);
  this->SetCameraModel(model);
}

int DepthMap::ExtractDepthMap(float threshold, int mask_size) {
  if (!point_cloud_initialized_ || !model_initialized_) {
    BEAM_CRITICAL("Variables not properly initialized.");
    throw std::runtime_error{"Variables not properly initialized."};
  }
  BEAM_INFO("Extracting Depth Image...");
  // create image mask where white pixels = projection hit
  cv::Mat hit_mask = beam_cv::CreateHitMask(mask_size, model_, cloud_);
  // create image with 3 channels for coordinates
  depth_image_ = std::make_shared<cv::Mat>(model_->GetHeight(),
                                           model_->GetWidth(), CV_32FC1);
  // perform ray casting of cloud to create depth_image_
  beam_cv::RayCast(depth_image_, cloud_, hit_mask, threshold, model_,
                   HitBehaviour);

  depth_image_extracted_ = true;
  int num_extracted = 0;
  // compute min and max depth in the image
  min_depth_ = 1000, max_depth_ = 0;
  depth_image_->forEach<float>(
      [&](float& distance, const int* position) -> void {
        (void)position;
        if (distance != 0.0) {
          num_extracted++;
          if (distance > max_depth_) { max_depth_ = distance; }
          if (distance < min_depth_) { min_depth_ = distance; }
        }
      });
  return num_extracted;
}

int DepthMap::ExtractDepthMapProjection(float thresh) {
  // create image with 3 channels for coordinates
  depth_image_ = std::make_shared<cv::Mat>(model_->GetHeight(),
                                           model_->GetWidth(), CV_32FC1);
  for (uint32_t i = 0; i < cloud_->points.size(); i++) {
    Eigen::Vector3d origin(0, 0, 0);
    Eigen::Vector3d point(cloud_->points[i].x, cloud_->points[i].y,
                          cloud_->points[i].z);
    opt<Eigen::Vector2i> coords = model_->ProjectPoint(point);
    if (!coords.has_value()) { continue; }
    // if successful projeciton calculate distance and fill depth image
    uint16_t col = coords.value()(0, 0), row = coords.value()(1, 0);
    float dist = beam::distance(point, origin);
    if ((depth_image_->at<float>(row, col) > dist ||
         depth_image_->at<float>(row, col) > dist) &&
        dist < thresh) {
      depth_image_->at<float>(row, col) = dist;
    }
  }
  depth_image_extracted_ = true;
  int num_extracted = 0;
  // compute min and max depth in the image
  min_depth_ = 1000, max_depth_ = 0;
  depth_image_->forEach<float>(
      [&](float& distance, const int* position) -> void {
        (void)position;
        if (distance != 0.0) {
          num_extracted++;
          if (distance > max_depth_) { max_depth_ = distance; }
          if (distance < min_depth_) { min_depth_ = distance; }
        }
      });
  return num_extracted;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DepthMap::ExtractPointCloud() {
  if (!this->CheckState()) {
    BEAM_CRITICAL("Variables not properly set.");
    throw std::runtime_error{"Variables not properly set."};
  }
  BEAM_INFO("Performing Point Cloud Construction...");
  pcl::PointCloud<pcl::PointXYZ>::Ptr dense_cloud =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (int row = 0; row < depth_image_->rows; row += 2) {
    for (int col = 0; col < depth_image_->cols; col += 2) {
      float distance = depth_image_->at<float>(row, col);
      if (distance > 0) {
        Eigen::Vector2i pixel(col, row);
        opt<Eigen::Vector3d> direction = model_->BackProject(pixel);
        if (direction.has_value()) {
          Eigen::Vector3d coords = distance * direction.value();
          pcl::PointXYZ point(coords[0], coords[1], coords[2]);
          dense_cloud->points.push_back(point);
        } else {
          BEAM_WARN("Pixel cannot be back projected, skipping.");
        }
      }
    }
  }
  dense_cloud->width = 1;
  dense_cloud->height = dense_cloud->points.size();
  return dense_cloud;
}

bool DepthMap::CheckState() {
  bool state = false;
  if (point_cloud_initialized_ && depth_image_extracted_ &&
      model_initialized_) {
    state = true;
  }
  return state;
}

Eigen::Vector3d DepthMap::GetXYZ(const Eigen::Vector2i& pixel) {
  float distance = depth_image_->at<float>(pixel[0], pixel[1]);
  if (distance == 0.0) {
    Eigen::Vector2i c = beam_depth::FindClosest(pixel, *depth_image_);
    distance = depth_image_->at<float>(c[0], c[1]);
  }
  opt<Eigen::Vector3d> direction = model_->BackProject(pixel);
  if (direction.has_value()) {
    Eigen::Vector3d coords = distance * direction.value();
    return coords;
  } else {
    BEAM_WARN("Pixel cannot be back projected, skipping.");
    return Eigen::Vector3d(0, 0, 0);
  }
}

float DepthMap::GetDistance(const Eigen::Vector2i& p1,
                            const Eigen::Vector2i& p2) {
  Eigen::Vector3d coord1 = GetXYZ(p1), coord2 = GetXYZ(p2);
  return beam::distance(coord1, coord2);
}

float DepthMap::GetPixelScale(const Eigen::Vector2i& pixel) {
  float distance = depth_image_->at<float>(pixel[0], pixel[1]);
  if (distance == 0.0) {
    Eigen::Vector2i c = beam_depth::FindClosest(pixel, *depth_image_);
    distance = depth_image_->at<float>(c[0], c[1]);
  }
  Eigen::Vector2i left(pixel[0], pixel[1] - 1), right(pixel[0], pixel[1] - 1);
  opt<Eigen::Vector3d> dir_left = model_->BackProject(left);
  opt<Eigen::Vector3d> dir_right = model_->BackProject(right);
  if (!dir_left.has_value() || !dir_right.has_value()) {
    BEAM_ERROR("Cannot get pixel scale. Pixel invalid, cannot back project.");
    return 0;
  }
  Eigen::Vector3d coords_left = distance * dir_left.value(),
                  coords_right = distance * dir_right.value();
  float area = beam::distance(coords_left, coords_right) *
               beam::distance(coords_left, coords_right);
  return area;
}

void DepthMap::Subsample(const float percentage_keep) {
  int gcd = beam::gcd(depth_image_->rows, depth_image_->cols);
  BEAM_INFO("Subsample window size: {}", gcd);
  for (int row = 0; row < depth_image_->rows; row++) {
    for (int col = 0; col < depth_image_->cols; col++) {
      if (row % gcd == 0 && col % gcd == 0) {
        // valid window start position
        std::vector<std::tuple<float, cv::Point2i>> window_depths;
        for (int window_row = row; window_row <= row + gcd; window_row++) {
          for (int window_col = col; window_col <= col + gcd; window_col++) {
            cv::Point2i p(window_row, window_col);
            float depth = depth_image_->at<float>(window_row, window_col);
            if (depth != 0) {
              window_depths.push_back(std::make_tuple(depth, p));
            }
          }
        }

        int step = 1 / percentage_keep;
        int count = 0;
        for (int i = 0; i < window_depths.size(); i += step) {
          if (i % step != 0) {
            cv::Point2i p = std::get<1>(window_depths[i]);
            depth_image_->at<float>(p.x, p.y) = 0;
            count++;
          }
        }
        BEAM_DEBUG("Points dropped in window: {}", count);
      }
    }
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DepthMap::GetCloud() {
  if (!point_cloud_initialized_) {
    BEAM_CRITICAL("Cloud not set.");
    throw std::runtime_error{"Cloud not set."};
  }
  return cloud_;
}

void DepthMap::SetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
  cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::copyPointCloud(*input_cloud, *cloud_);
  point_cloud_initialized_ = true;
}

cv::Mat DepthMap::GetDepthImage() {
  if (!depth_image_extracted_) {
    BEAM_CRITICAL("Depth image not extracted.");
    throw std::runtime_error{"Depth image not extracted."};
  }
  return *depth_image_;
}

void DepthMap::SetDepthImage(cv::Mat input) {
  depth_image_ = std::make_shared<cv::Mat>(input);
  depth_image_extracted_ = true;
  min_depth_ = 1000, max_depth_ = 0;
  depth_image_->forEach<float>(
      [&](float& distance, const int* position) -> void {
        (void)position;
        if (distance != 0.0) {
          if (distance > max_depth_) { max_depth_ = distance; }
          if (distance < min_depth_) { min_depth_ = distance; }
        }
      });
}

std::shared_ptr<beam_calibration::CameraModel> DepthMap::GetCameraModel() {
  if (!model_initialized_) {
    BEAM_CRITICAL("Camera model not set.");
    throw std::runtime_error{"Camera model not set."};
  }
  return model_;
}

void DepthMap::SetCameraModel(
    std::shared_ptr<beam_calibration::CameraModel> input_model) {
  model_ = input_model;
  model_initialized_ = true;
}

} // namespace beam_depth
