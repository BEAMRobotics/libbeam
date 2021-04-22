#include <beam_matching/loam/LoamFeatureExtractor.h>

#include <Eigen/Geometry>

#include <beam_utils/angles.h>
#include <beam_utils/log.h>

namespace beam_matching {

LoamFeatureExtractor::LoamFeatureExtractor(const LoamParamsPtr& params)
    : params_(params) {}

LoamPointCloud LoamFeatureExtractor::ExtractFeatures(const PointCloud& cloud) {
  GetScanLines(cloud);
  return LoamPointCloud();
}

void LoamFeatureExtractor::GetScanLines(const PointCloud& cloud){
  ScanLines_ = std::vector<PointCloud>(params_->number_of_beams, PointCloud());

  // calculate bins for angle of beams
  std::vector<double> beam_angle_bins_deg = params_->GetBeamAngleBinsDeg();

  // extract valid points from input cloud
  for (auto point_id = 0; point_id < cloud.size(); point_id++) {
    const pcl::PointXYZ& point = cloud[point_id];

    // skip NaN and INF valued points
    if (!pcl_isfinite(point.x) || !pcl_isfinite(point.y) ||
        !pcl_isfinite(point.z)) {
      continue;
    }

    // skip zero valued points
    if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001) {
      continue;
    }

    // calculatepoint's vertical angle
    float angle_deg = beam::Rad2Deg(
        std::atan(point.y / std::sqrt(point.x * point.x + point.z * point.z)));

    // get the scan bin (or scan line)
    double max_angle = params_->fov_deg / 2;
    double min_angle = -max_angle;
    int line_id = beam_angle_bins_deg.size() - 1;
    for (int i = 0; i < beam_angle_bins_deg.size(); i++) {
      if (angle_deg > beam_angle_bins_deg[i]) {
        line_id = i;
        break;
      }
    }
    ScanLines_[line_id].push_back(point);
  }
}

} // namespace beam_matching
