#include "beam_cv/geometry/AbsolutePoseEstimator.h"
#include "beam_cv/geometry/Triangulation.h"
#include "beam_utils/math.hpp"

#include <Eigen/Geometry>

#include <chrono>
#include <cstdlib>

namespace beam_cv {
std::vector<Eigen::Matrix4d>
    P3PEstimator(std::shared_ptr<beam_calibration::CameraModel> cam,
                 std::vector<Eigen::Vector2i> pixels,
                 std::vector<Eigen::Vector3d> points) {
  //normalize pixels coords
  std::vector<Eigen::Vector3d> yi;
  for (std::size_t i = 0; i<pixels.size(); ++i) {
    Eigen::Vector3d pixelNorm(pixels[i][0], pixels[i][1], 1);
    pixelNorm.normalize();
    yi.push_back(pixelNorm);
  }
  //compute aij and bij

  //construct D1 and D2

  //compute a real root of the cubic equation

  // D0 = D1 + gamma D2

  // solve for the two non-zero eigen values of D0

  // 
  std::vector<Eigen::Matrix4d> transformations;

  return transformations;
}
} // namespace beam_cv