#include <beam_calibration/CameraModels.h>

#include <chrono>
#include <ctime>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

#include <beam_utils/log.h>
#include <beam_utils/pointclouds.h>

namespace beam_calibration {

std::shared_ptr<CameraModel> CameraModel::Create(std::string& file_location) {
  std::shared_ptr<CameraModel> camera_model;

  // check file exists
  if (!boost::filesystem::exists(file_location)) {
    BEAM_CRITICAL("Invalid file path for camera intialization. Input: {}",
                  file_location);
    throw std::runtime_error{"Invalid file path for camera intialization."};
  }

  std::string file_ext = boost::filesystem::extension(file_location);
  if (file_ext == ".conf") {
    BEAM_ERROR("Cannot use Create() factory method with ladybug model.");
    throw std::runtime_error{"Invalid camera type."};
  } else if (file_ext == ".json") {
    // load JSON
    nlohmann::json J;
    std::ifstream file(file_location);
    file >> J;
    std::string camera_type = J["camera_type"];
    if (camera_type == "KANNALABRANDT") {
      camera_model = std::make_shared<KannalaBrandt>(file_location);
    } else if (camera_type == "DOUBLESPHERE") {
      camera_model = std::make_shared<DoubleSphere>(file_location);
    } else if (camera_type == "RADTAN") {
      camera_model = std::make_shared<Radtan>(file_location);
    } else if (camera_type == "CATADITROPIC") {
      camera_model = std::make_shared<Cataditropic>(file_location);
    } else {
      BEAM_CRITICAL("Invalid camera type read from JSON.");
      throw std::runtime_error{"Invalid camera type read from JSON."};
    }
  } else {
    BEAM_CRITICAL("Invalid file type read for camera intialization.");
    throw std::runtime_error{
        "Invalid file type read for camera intialization."};
  }

  return camera_model;
}

void CameraModel::InitUndistortMap() {
  if(undistort_map_initialized_){
    return;
  }
  BEAM_INFO("Creating undistortion map...");
  // get the rectified model
  GetRectifiedModel();
  int height = GetHeight();
  int width = GetWidth();
  // pixel_map_.at(distorted_pixel) = undistorted_pixel
  pixel_map_ = std::make_shared<cv::Mat>(height, width, CV_32SC2);
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      Eigen::Vector3d point_back_projected;
      if (!BackProject(Eigen::Vector2i(j, i), point_back_projected)) {
        (*pixel_map_).at<cv::Vec2i>(i, j).val[0] = -999999;
        (*pixel_map_).at<cv::Vec2i>(i, j).val[1] = -999999;
        continue;
      }

      Eigen::Vector2d point_projected;
      if (!rectified_model_->ProjectPoint(point_back_projected,
                                          point_projected)) {
        (*pixel_map_).at<cv::Vec2i>(i, j).val[0] = -999999;
        (*pixel_map_).at<cv::Vec2i>(i, j).val[1] = -999999;
        continue;
      } else {
        // we still allow pixels outside of the image plane to be undistorted
        (*pixel_map_).at<cv::Vec2i>(i, j).val[0] = point_projected[0];
        (*pixel_map_).at<cv::Vec2i>(i, j).val[1] = point_projected[1];
      }
    }
  }
  undistort_map_initialized_ = true;
  BEAM_INFO("Done.");
}

bool CameraModel::UndistortPixel(const Eigen::Vector2i& in_pixel,
                                 Eigen::Vector2i& out_pixel) {
  // create undistort map if it doesnt exist
  if (!pixel_map_) { InitUndistortMap(); }

  // add check for invalid input
  if (in_pixel[1] >= (int)GetHeight() || in_pixel[0] >= (int)GetWidth() ||
      in_pixel[1] < 0 || in_pixel[0] < 0) {
    return false;
  } else {
    cv::Vec2i out = (*pixel_map_).at<cv::Vec2i>(in_pixel[1], in_pixel[0]);
    if (out[0] == -999999 || out[1] == -999999) {
      return false;
    } else {
      out_pixel = Eigen::Vector2i(out[0], out[1]);
    }
  }
  return true;
}

std::shared_ptr<CameraModel> CameraModel::GetRectifiedModel() {
  if (!rectified_model_) {
    Eigen::Matrix<double, 8, 1> intrinsics;
    intrinsics(0, 0) = GetIntrinsics()[0];
    intrinsics(1, 0) = GetIntrinsics()[1];
    intrinsics(2, 0) = GetWidth() / 2;
    intrinsics(3, 0) = GetHeight() / 2;
    intrinsics(4, 0) = 0;
    intrinsics(5, 0) = 0;
    intrinsics(6, 0) = 0;
    intrinsics(7, 0) = 0;

    rectified_model_ =
        std::make_shared<Radtan>(GetHeight(), GetWidth(), intrinsics);
    rectified_model_->SetFrameID(GetFrameID());
    rectified_model_->SetSafeProjectionRadius(GetSafeProjectionRadius());
  }
  return rectified_model_;
}

void CameraModel::SetCameraID(const unsigned int id) {
  cam_id_ = id;
}

void CameraModel::SetFrameID(const std::string& id) {
  frame_id_ = id;
}

void CameraModel::SetCalibrationDate(const std::string& date) {
  calibration_date_ = date;
}

void CameraModel::SetImageDims(const uint32_t height, const uint32_t width) {
  image_width_ = width;
  image_height_ = height;
}

Eigen::Matrix3d CameraModel::GetIntrinsicMatrix() {
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  K(0, 0) = intrinsics_[0];
  K(1, 1) = intrinsics_[1];
  K(0, 2) = intrinsics_[2];
  K(1, 2) = intrinsics_[3];
  return K;
}

void CameraModel::SetSafeProjectionRadius(uint32_t safe_projection_radius) {
  safe_projection_radius_ = safe_projection_radius;
}

uint32_t CameraModel::GetSafeProjectionRadius() {
  return safe_projection_radius_;
}

void CameraModel::SetIntrinsics(const Eigen::VectorXd& intrinsics) {
  if (intrinsics.size() != intrinsics_size_[GetType()]) {
    BEAM_CRITICAL("Invalid number of elements in intrinsics vector.");
    throw std::runtime_error{
        "Invalid number of elements in intrinsics vector."};
  } else {
    intrinsics_ = intrinsics;
  }
}

const std::string CameraModel::GetFrameID() const {
  return frame_id_;
}

const std::string CameraModel::GetCalibrationDate() const {
  if (calibration_date_ == "") { BEAM_WARN("Calibration date empty."); }
  return calibration_date_;
}

uint32_t CameraModel::GetHeight() const {
  if (image_height_ == 0) { BEAM_WARN("Image height not set."); }
  return image_height_;
}

uint32_t CameraModel::GetWidth() const {
  if (image_width_ == 0) { BEAM_WARN("Image width not set."); }
  return image_width_;
}

const Eigen::VectorXd& CameraModel::GetIntrinsics() const {
  return intrinsics_;
}

CameraType CameraModel::GetType() const {
  return type_;
}

bool CameraModel::PixelInImage(const Eigen::Vector2i& pixel) {
  Eigen::Vector2d pixel_d = pixel.cast<double>();
  return PixelInImage(pixel_d);
}

bool CameraModel::PixelInImage(const Eigen::Vector2d& pixel) {
  // check if in image plane
  if (pixel[0] < 0 || pixel[1] < 0 || pixel[0] > image_width_ - 1 ||
      pixel[1] > image_height_ - 1) {
    return false;
  }

  // check if in safe projection radius
  if (safe_projection_radius_ == 0) { return true; }

  double distance_from_center =
      std::sqrt((pixel[0] - intrinsics_[2]) * (pixel[0] - intrinsics_[2]) +
                (pixel[1] - intrinsics_[3]) * (pixel[1] - intrinsics_[3]));
  if (distance_from_center > safe_projection_radius_) { return false; }

  // else, it's in the valid image range
  return true;
}

void CameraModel::LoadJSON(const std::string& file_location) {
  // load file
  nlohmann::json J;
  std::ifstream file(file_location);
  file >> J;
  // get string repr of class type
  std::string class_type;
  for (std::map<std::string, CameraType>::iterator it =
           intrinsics_types_.begin();
       it != intrinsics_types_.end(); it++) {
    if (intrinsics_types_[it->first] == type_) { class_type = it->first; }
  }

  // check type
  std::string camera_type = J["camera_type"];
  std::map<std::string, CameraType>::iterator it =
      intrinsics_types_.find(camera_type);
  if (it == intrinsics_types_.end()) {
    BEAM_CRITICAL("Invalid camera type read from json. Type read: {}",
                  camera_type.c_str());
    OutputCameraTypes();
  } else if (intrinsics_types_[camera_type] != type_) {
    BEAM_CRITICAL("Camera type read from JSON does not match expected type. "
                  "Type read: {}, Expected: {}",
                  camera_type.c_str(), class_type.c_str());
  }
  // get params
  calibration_date_ = J["date"];
  image_width_ = J["image_width"];
  image_height_ = J["image_height"];
  frame_id_ = J["frame_id"];
  std::vector<double> intrinsics;
  for (const auto& value : J["intrinsics"]) {
    intrinsics.push_back(value.get<double>());
  }

  intrinsics_ = Eigen::VectorXd::Map(intrinsics.data(), intrinsics.size());
  if (intrinsics_.size() != intrinsics_size_[type_]) {
    BEAM_CRITICAL("Invalid number of intrinsics read. read: {}, required: {}",
                  intrinsics_.size(), intrinsics_size_[type_]);
    throw std::invalid_argument{"Invalid number of instrinsics read."};
  }

  if (J.contains("safe_projection_radius")) {
    safe_projection_radius_ = J["safe_projection_radius"];
  }
}

void CameraModel::WriteJSON(const std::string& file_location,
                            const std::string& method) {
  BEAM_INFO("Writing to file: {}", file_location);

  std::time_t date =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::string cur_date = std::string(std::ctime(&date));
  // load file
  nlohmann::json J;
  J["date"] = cur_date;
  if (method.empty()) {
    J["method"] = std::string("unkown");
  } else {
    J["method"] = method;
  }
  // get string repr of class type
  std::string class_type;
  for (std::map<std::string, CameraType>::iterator it =
           intrinsics_types_.begin();
       it != intrinsics_types_.end(); it++) {
    if (intrinsics_types_[it->first] == type_) { class_type = it->first; }
  }
  J["camera_type"] = class_type;
  J["image_width"] = this->GetWidth();
  J["image_height"] = this->GetHeight();
  J["frame_id"] = this->GetFrameID();
  Eigen::VectorXd intrinsics_eigen = this->GetIntrinsics();
  std::vector<double> intrinsics_vec(&intrinsics_eigen[0],
                                     intrinsics_eigen.data() +
                                         intrinsics_eigen.cols() *
                                             intrinsics_eigen.rows());
  J["intrinsics"] = intrinsics_vec;
  std::ofstream out(file_location);
  out << std::setw(4) << J << std::endl;
}

void CameraModel::OutputCameraTypes() {
  std::cout << "Intrinsic type input options:\n";
  for (std::map<std::string, CameraType>::iterator it =
           intrinsics_types_.begin();
       it != intrinsics_types_.end(); it++) {
    std::cout << "    -" << it->first << "\n";
  }
}

pcl::PointCloud<pcl::PointXYZRGBL>
    CameraModel::CreateCameraFrustum(const ros::Time& t, double increment,
                                     double length,
                                     const Eigen::Vector3i& RGB) {
  // draw info needed to draw camera frustums
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> rays;
  std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>>
      pixels;
  pixels.emplace_back(0, 0);
  pixels.emplace_back(0, image_height_);
  pixels.emplace_back(image_width_, image_height_);
  pixels.emplace_back(image_width_, 0);

  for (int i = 0; i < 4; i++) {
    Eigen::Vector3d ray;
    if (!BackProject(pixels[i], ray)) {
      BEAM_ERROR("cannot back project ray");
      break;
    }
    rays.push_back(ray / ray.norm());
  }

  pcl::PointXYZRGBL point_start(0, 0, 0, RGB[0], RGB[1], RGB[2], t.toSec());
  pcl::PointCloud<pcl::PointXYZRGBL> frustum;

  // draw 4 corner rays and store end points
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      end_points;
  for (int i = 0; i < 4; i++) {
    Eigen::Vector3d point_end_eig = rays[i] * length;
    pcl::PointXYZRGBL point_end = point_start;
    point_end.x = point_end_eig[0];
    point_end.y = point_end_eig[1];
    point_end.z = point_end_eig[2];
    pcl::PointCloud<pcl::PointXYZRGBL> line =
        beam::DrawLine<pcl::PointXYZRGBL>(point_start, point_end, increment);
    frustum += line;
    end_points.push_back(point_end_eig);
  }

  // now connect the end points
  for (int i = 0; i < 3; i++) {
    pcl::PointXYZRGBL pt1 = point_start;
    pt1.x = end_points[i][0];
    pt1.y = end_points[i][1];
    pt1.z = end_points[i][2];
    pcl::PointXYZRGBL pt2 = point_start;
    pt2.x = end_points[i + 1][0];
    pt2.y = end_points[i + 1][1];
    pt2.z = end_points[i + 1][2];
    pcl::PointCloud<pcl::PointXYZRGBL> line =
        beam::DrawLine<pcl::PointXYZRGBL>(pt1, pt2, increment);
    frustum += line;
  }

  // draw final line
  pcl::PointXYZRGBL pt1 = point_start;
  pt1.x = end_points[0][0];
  pt1.y = end_points[0][1];
  pt1.z = end_points[0][2];
  pcl::PointXYZRGBL pt2 = point_start;
  pt2.x = end_points[3][0];
  pt2.y = end_points[3][1];
  pt2.z = end_points[3][2];
  pcl::PointCloud<pcl::PointXYZRGBL> line =
      beam::DrawLine<pcl::PointXYZRGBL>(pt1, pt2, increment);
  frustum += line;
  return frustum;
}

} // namespace beam_calibration
