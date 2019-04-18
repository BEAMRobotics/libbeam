/** @file
 * @ingroup containers
 * Image data container for bridge type inspections
 *
 * @defgroup images
 * Custom data containers for inspection specific objects. E.g., Image
 * containers with defect masks, special point types for pcl point clouds.
 */

#pragma once

#include <beam/utils/time.hpp>
#include <fstream>
#include <string>
#include <Eigen/Dense>

namespace beam_containers {
/** @addtogroup containers
 *  @{ */

/**
 * @brief read a pose file
 * @return poses Vector of poses containing timepoints and transformations
 * @param poses_file
 */
std::vector<std::pair<beam::TimePoint, Eigen::Affine3d>>
    ReadPoseFile(std::string poses_file) {
  std::ifstream file;
  std::string line;
  Eigen::Matrix4d Tk;
  Eigen::Affine3d TAk;
  uint64_t tk;
  beam::TimePoint time_point_k;
  std::vector<std::pair<beam::TimePoint, Eigen::Affine3d>> poses;

  file.open(poses_file);

  while (!file.eof()) {
    std::getline(file, line, ',');
    try {
      tk = std::stod(line);
    } catch (const std::invalid_argument& e) {
      // Invalid argument, probably at end of file
      return poses;
    }

    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        if (i == 3 && j == 3) {
          std::getline(file, line, '\n');
          Tk(i, j) = std::stod(line);
        } else {
          std::getline(file, line, ',');
          Tk(i, j) = std::stod(line);
        }
      }
    }
    beam::TimePoint time_point_k{std::chrono::duration_cast<beam::TimePoint::duration>(
        std::chrono::nanoseconds(tk))};

    // time_point_k{std::chrono::duration_cast<beam::TimePoint::duration>(
    //     std::chrono::nanoseconds(tk))};
    TAk.matrix() = Tk;
    poses.push_back(std::make_pair(time_point_k, TAk));
  }
  return poses;
}

/** @} group containers */

} // namespace beam_containers
