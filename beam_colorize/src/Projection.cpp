#include "beam/colorize/Projection.h"

namespace beam_colorize {

std::pair<double, double> Projection::XYZToUVRect(int cam, double x, double y,
                                                  double z) const {
  double focal_length_ = 0;  // cameras_[cam].f;
  double rect_center_u_ = 0; // cameras_[cam].cx;
  double rect_center_v_ = 0; // cameras_[cam].cy;

  double rect_cols_ = 0; // FULL_HEIGHT; // 2460;
  double rect_rows_ = 0; // FULL_WIDTH;  // 2040;

  double u = -1;
  double v = -1;

  u = ((focal_length_)*x / z +
       rect_center_u_); // This projection is into the natural rectified image.
  v = ((focal_length_)*y / z + rect_center_v_);

  // Make sure point is in rectified image
  if (u < 0 || u > rect_cols_ - 1 || v < 0 || v > rect_rows_ - 1) {
    u = -1;
    v = -1;
  }

  return std::make_pair(u, v);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Projection::ColorizePointCloud(
    const pcl::PointCloud<pcl::PointXYZ>& input_cloud,
    const sensor_msgs::Image& input_image) const {
  return pcl::PointCloud<pcl::PointXYZRGB>::Ptr();
}

} // namespace beam_colorize