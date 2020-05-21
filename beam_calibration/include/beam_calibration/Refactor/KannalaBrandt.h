/** @file
 * @ingroup calibration
 */

#pragma once

#include "beam_calibration/include/Refactor/CameraModel.h"

namespace beam_calibration {

/**
 * @brief Derived class for pinhole camera model
 */
class KannalaBrandt : public CameraModel {
public:
  /**
   * @brief Constructor
   * @param input_file path to input file
   */
  KannalaBrandt(const std::string& file_path);

  /**
   * @brief Default destructor
   */
  KannalaBrandt();

  /**
   * @brief Default destructor
   */
  ~KannalaBrandt() = default;

  /**
   * @brief Method for projecting a point into an image plane
   * @param point 3d point to be projected [x,y,z]^T
   * @param pixel reference to an optional vector with image coordinates after
   * point has been projected into the image plane [u,v]^T
   */
  void ProjectPoint(const Eigen::Vector3d& point,
                    std::optional<Eigen::Vector2i>& pixel) override;

  /**
   * @brief Overload projection function for computing jacobian of projection
   * @param point 3d point to be projected [x,y,z]^T
   * @param pixel reference to an optional vector with image coordinates after
   * point has been projected into the image plane [u,v]^T
   * @param J 2 x 3 projection jacobian.
   * For ProjectPoint: [u,v]^T = [P1(x, y, z), P2(x, y, z)]^T
   *                   J = | dP1/dx , dP1/dy, dP1/dz |
   *                       | dP2/dx , dP2/dy, dP2/dz |
   */
  void ProjectPoint(const Eigen::Vector3d& point,
                    std::optional<Eigen::Vector2i>& pixel,
                    std::optional<Eigen::MatrixXd>& J) override;

  /**
   * @brief Method back projecting
   * @return Returns bearing vector
   * @param point [u,v]
   */
  void BackProject(const Eigen::Vector2i& pixel,
                   std::optional<Eigen::Vector3d>& ray) override;

  /**
   * @brief Method for validating the inputs. This will be called in the load
   * configuration file step and should validate the intrinsics (i.e. size) and
   * the type
   */
  void ValidateInputs() override;

protected:
  /**
   * @brief Method for loading calibration information from a json.
   * @param file_path full path to json
   */
  void LoadJSON(const std::string& file_path);
};

/** @} group calibration */

} // namespace beam_calibration
