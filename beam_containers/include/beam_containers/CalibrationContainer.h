/** @file
 * @ingroup containers
 * Calibration container to store all of a robots calibrations
 *
 */

#pragma once

#include <beam_calibration/CameraModel.h>
#include <beam_calibration/TfTree.h>
#include <beam_utils/utils.hpp>

#include <nlohmann/json.hpp>

#include <fstream>

using namespace std;
using namespace beam_calibration;

namespace beam_containers {
/** @addtogroup containers
 *  @{ */

struct Camera {
  size_t camera_id;
  shared_ptr<CameraModel> intrinsics;
  // frame is stored in camera intrinsics: intrinsics.GetFrameID()
}

struct Imu {
  size_t id;
  std::string frame;
  Eigen::MatrixXd<double, 5, 1> intrinsics;
}

struct Lidar {
  size_t id;
  std::string frame;
}

/**
 * @brief class for bridge type image container
 */
class CalibrationContainer {
public:
  /**
   * @brief Default constructor
   */
  CalibrationContainer() = default;

  /**
   * @brief Custom constructor
   * @param cameras vector of camera models to initialize with
   * @param imus vector of (frame, intrinsics) representing imus
   * @param lidars vector of frames associated to lidars
   * @param baselink_frame frame of robots base
   */
  CalibrationContainer(
      vector<shared_ptr<CameraModel>>& cameras,
      vector<tuple<string, Eigen::MatrixXd<double, 5, 1>>>& imus,
      vector<string>& lidars, TfTree& extrinsics, std::string baselink_frame) {}

  /**
   * @brief Default destructor
   */
  virtual ~CalibrationContainer() = default;

  /**
   * @brief Loads calibrations from a specified folder
   * @param path_to_calibration folder path to load from
   */
  void LoadCalibrations(const string& path_to_calibration) {}

  /**
   * @brief Saves current calibrations to a folder
   * @param path_to_calibration folder path to save to
   */
  void SaveCalibrations(const string& path_to_calibration) {}

  /**
   * @brief Gets camera model, by id
   * @param id to search for
   */
  shared_ptr<CameraModel> GetCameraIntrinsics(size_t id) {}

  /**
   * @brief Gets camera model, by id
   * @param id to search for
   */
  shared_ptr<CameraModel> GetIMUIntrinsics(size_t id) {}

  /**
   * @brief Gets list of id's associated to cameras
   */
  vector<size_t> GetCameraIds() {}

  /**
   * @brief Gets list of id's associated to IMU's
   */
  vector<size_t> GetIMUIds() {}

  /**
   * @brief Gets list of id's associated to Lidars
   */
  vector<size_t> GetLidarIds() {}

  /**
   * @brief Gets transform from base frame to sensor frame, by id
   * @param id to search for
   */
  Eigen::Matrix4d T_Sensor_Baselink(size_t id) {}

private:
  // current value of the uuid used for sensor maps
  size_t cur_uuid_ = 0;
  /* Sensor maps:
   * The id's for the maps should be unique across all three i.e.,
   * Querying imus_[1] should return nothing if 1 is a key in cameras_
   */
  unordered_map<size_t, Camera> cameras_;
  unordered_map<size_t, Imu> imus_;
  unordered_map<size_t, Lidar> lidars_;
  // Object storing extrinsics: see beam_calibration/TfTree.h
  TfTree extrinsics_;
  // String of the robots base frame (typically base_link or hvlp_link)
  std::string baselink_frame_;
};

/** @} group containers */

} // namespace beam_containers
