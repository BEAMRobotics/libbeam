#include <beam_utils/simple_path_generator.h>

#include <math.h>

namespace beam {

SimplePathGenerator::SimplePathGenerator(
    const std::vector<Eigen::Vector3d,
                      Eigen::aligned_allocator<Eigen::Vector3d>>& nodes,
    double delta)
    : delta_(delta) {
  Eigen::MatrixXd points(3, nodes.size());
  for (size_t i = 0; i < nodes.size(); i++) {
    points.col(i) << nodes[i][0], nodes[i][1], nodes[i][2];
  }

  spline_ = Eigen::SplineFitting<Eigen::Spline3d>::Interpolate(
      points, nodes.size() - 1);
}

Eigen::Matrix4d SimplePathGenerator::GetPose(double p) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

  // get interpolated point
  Eigen::VectorXd x_query = spline_(p);
  T(0, 3) = x_query[0];
  T(1, 3) = x_query[1];
  T(2, 3) = x_query[2];

  // get point marginally past query point and calculate yaw
  Eigen::VectorXd x_query_plus = spline_(p + delta_);
  double dx = x_query[0] - x_query_plus[0];
  double dy = x_query[1] - x_query_plus[1];
  double yaw = std::atan2(dy, dx);

  // calculate rotation matrix
  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  T.block(0, 0, 3, 3) = R;

  return T;
}

} // namespace beam
