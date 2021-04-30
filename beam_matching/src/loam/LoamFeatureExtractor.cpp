#include <beam_matching/loam/LoamFeatureExtractor.h>

#include <Eigen/Geometry>
#include <pcl/filters/voxel_grid.h>

#include <beam_utils/angles.h>
#include <beam_utils/log.h>

namespace beam_matching {

LoamFeatureExtractor::LoamFeatureExtractor(const LoamParamsPtr& params)
    : params_(params) {}

LoamPointCloud LoamFeatureExtractor::ExtractFeatures(const PointCloud& cloud) {
  Reset();
  std::vector<PointCloud> scan_lines = GetScanLines(cloud);
  GetSortedScan(scan_lines);

  // extract features from individual scans
  for (size_t i = 0; i < scan_indices_.size(); i++) {
    PointCloudPtr surf_points_less_flat_scan = std::make_shared<PointCloud>();
    size_t scan_start_idx = scan_indices_[i].first;
    size_t scan_end_idx = scan_indices_[i].second;

    // skip empty scans
    if (scan_end_idx <= scan_start_idx + 2 * params_->curvature_region) {
      continue;
    }

    // reset scan buffers
    SetScanBuffersFor(scan_start_idx, scan_end_idx);

    // extract features from equally sized scan regions
    for (int j = 0; j < params_->n_feature_regions; j++) {
      size_t sp = ((scan_start_idx + params_->curvature_region) *
                       (params_->n_feature_regions - j) +
                   (scan_end_idx - params_->curvature_region) * j) /
                  params_->n_feature_regions;
      size_t ep = ((scan_start_idx + params_->curvature_region) *
                       (params_->n_feature_regions - 1 - j) +
                   (scan_end_idx - params_->curvature_region) * (j + 1)) /
                      params_->n_feature_regions -
                  1;

      // skip empty regions
      if (ep <= sp) { continue; }

      size_t region_size = ep - sp + 1;

      // reset region buffers
      SetRegionBuffersFor(sp, ep);

      // extract corner features
      int largest_picked_num = 0;
      for (size_t k = region_size;
           k > 0 && largest_picked_num < params_->max_corner_less_sharp;) {
        size_t idx = region_sort_indices_[--k];
        size_t scan_idx = idx - scan_start_idx;
        size_t region_idx = idx - sp;

        if (scan_neighbor_picked_[scan_idx] == 0 &&
            region_curvature_[region_idx] >
                params_->surface_curvature_threshold) {
          largest_picked_num++;
          if (largest_picked_num <= params_->max_corner_sharp) {
            region_label_[region_idx] = PointLabel::CORNER_SHARP;
            corner_points_sharp_.push_back(sorted_scan_[idx]);
          } else {
            region_label_[region_idx] = PointLabel::CORNER_LESS_SHARP;
          }
          corner_points_less_sharp_.push_back(sorted_scan_[idx]);

          MarkAsPicked(idx, scan_idx);
        }
      }

      // extract flat surface features
      int smallest_picked_num = 0;
      for (size_t k = 0;
           k < region_size && smallest_picked_num < params_->max_surface_flat;
           k++) {
        size_t idx = region_sort_indices_[k];
        size_t scan_idx = idx - scan_start_idx;
        size_t region_idx = idx - sp;

        if (scan_neighbor_picked_[scan_idx] == 0 &&
            region_curvature_[region_idx] <
                params_->surface_curvature_threshold) {
          smallest_picked_num++;
          region_label_[region_idx] = PointLabel::SURFACE_FLAT;
          surface_points_flat_.push_back(sorted_scan_[idx]);

          MarkAsPicked(idx, scan_idx);
        }
      }

      // extract less flat surface features
      for (size_t k = 0; k < region_size; k++) {
        if (region_label_[k] <= PointLabel::SURFACE_LESS_FLAT) {
          surf_points_less_flat_scan->push_back(sorted_scan_[sp + k]);
        }
      }
    }

    // down size less flat surface point cloud of current scan
    PointCloud surf_points_less_flat_scanDS;
    pcl::VoxelGrid<pcl::PointXYZ> down_size_filter;
    down_size_filter.setInputCloud(surf_points_less_flat_scan);
    down_size_filter.setLeafSize(params_->less_flat_filter_size,
                                 params_->less_flat_filter_size,
                                 params_->less_flat_filter_size);
    down_size_filter.filter(surf_points_less_flat_scanDS);

    surface_points_less_flat_ += surf_points_less_flat_scanDS;
  }

  return LoamPointCloud(corner_points_sharp_, surface_points_flat_,
                        corner_points_less_sharp_, surface_points_less_flat_);
}

void LoamFeatureExtractor::Reset() {
  scan_indices_.clear();
  sorted_scan_.clear();
  region_curvature_.clear();
  region_label_.clear();
  region_sort_indices_.clear();
  scan_neighbor_picked_.clear();
  corner_points_sharp_.clear();
  corner_points_less_sharp_.clear();
  surface_points_flat_.clear();
  surface_points_less_flat_.clear();
}

std::vector<PointCloud>
    LoamFeatureExtractor::GetScanLines(const PointCloud& cloud) {
  std::vector<PointCloud> scan_lines(params_->number_of_beams, PointCloud());

  // calculate bins for angle of beams
  std::vector<double> beam_angle_bins_deg = params_->GetBeamAngleBinsDeg();

  // extract valid points from input cloud
  for (size_t point_id = 0; point_id < cloud.size(); point_id++) {
    const pcl::PointXYZ& point = cloud[point_id];

    // skip NaN and INF valued points
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }

    // skip zero valued points
    if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001) {
      continue;
    }

    // calculatepoint's vertical angle
    float angle_deg;
    if (params_->vertical_axis == "Z" || params_->vertical_axis == "z") {
      angle_deg = beam::Rad2Deg(std::atan(
          point.z / std::sqrt(point.x * point.x + point.y * point.y)));
    } else if (params_->vertical_axis == "Y" || params_->vertical_axis == "y") {
      angle_deg = beam::Rad2Deg(std::atan(
          point.y / std::sqrt(point.x * point.x + point.z * point.z)));
    } else if (params_->vertical_axis == "X" || params_->vertical_axis == "x") {
      angle_deg = beam::Rad2Deg(std::atan(
          point.x / std::sqrt(point.y * point.y + point.z * point.z)));
    } else {
      BEAM_ERROR("Invalid vertical axis param. Options: X, Y, Z");
      throw std::invalid_argument{"Invalid vertical axis param"};
    }

    // get the scan bin (or scan line)
    int line_id = beam_angle_bins_deg.size();
    for (size_t i = 0; i < beam_angle_bins_deg.size(); i++) {
      if (angle_deg > beam_angle_bins_deg[i]) {
        line_id = i;
        break;
      }
    }
    scan_lines[line_id].push_back(point);
  }

  // Save results
  if (output_scan_lines_ && cloud.size() > 0) {
    for (size_t i = 0; i < scan_lines.size(); i++) {
      if (scan_lines[i].size() == 0) { continue; }
      pcl::io::savePCDFileASCII(debug_output_path_ + "scan" +
                                    std::to_string(i) + ".pcd",
                                scan_lines[i]);
    }
    pcl::io::savePCDFileASCII(debug_output_path_ + "scan_orig.pcd", cloud);
  }

  return scan_lines;
}

void LoamFeatureExtractor::GetSortedScan(
    const std::vector<PointCloud>& scan_lines) {
  size_t cloud_size = 0;
  for (size_t i = 0; i < scan_lines.size(); i++) {
    sorted_scan_ += scan_lines[i];

    IndexRange range(cloud_size, 0);
    cloud_size += scan_lines[i].size();
    range.second = cloud_size > 0 ? cloud_size - 1 : 0;
    scan_indices_.push_back(range);
  }

  // Save results
  if (output_scan_lines_ && sorted_scan_.size() > 0) {
    pcl::io::savePCDFileASCII(debug_output_path_ + "scan_sorted.pcd",
                              sorted_scan_);
  }
}

void LoamFeatureExtractor::SetScanBuffersFor(size_t start_idx, size_t end_idx) {
  // resize buffers
  size_t scan_size = end_idx - start_idx + 1;
  scan_neighbor_picked_.assign(scan_size, 0);

  // mark unreliable points as picked
  for (size_t i = start_idx + params_->curvature_region;
       i < end_idx - params_->curvature_region; i++) {
    const pcl::PointXYZ& previous_point = sorted_scan_[i - 1];
    const pcl::PointXYZ& point = sorted_scan_[i];
    const pcl::PointXYZ& next_point = sorted_scan_[i + 1];

    float diff_next = beam::SquaredDiff<pcl::PointXYZ>(next_point, point);

    if (diff_next > 0.1) {
      float depth1 = beam::PointDistance<pcl::PointXYZ>(point);
      float depth2 = beam::PointDistance<pcl::PointXYZ>(next_point);

      if (depth1 > depth2) {
        float weighted_distance = std::sqrt(beam::SquaredDiff<pcl::PointXYZ>(
                                      next_point, point, depth2 / depth1)) /
                                  depth2;

        if (weighted_distance < 0.1) {
          std::fill_n(
              &scan_neighbor_picked_[i - start_idx - params_->curvature_region],
              params_->curvature_region + 1, 1);
          continue;
        }
      } else {
        float weighted_distance = std::sqrt(beam::SquaredDiff<pcl::PointXYZ>(
                                      point, next_point, depth1 / depth2)) /
                                  depth1;

        if (weighted_distance < 0.1) {
          std::fill_n(&scan_neighbor_picked_[i - start_idx + 1],
                      params_->curvature_region + 1, 1);
        }
      }
    }

    float diffPrevious =
        beam::SquaredDiff<pcl::PointXYZ>(point, previous_point);
    float dis = beam::SquaredPointDistance<pcl::PointXYZ>(point);

    if (diff_next > 0.0002 * dis && diffPrevious > 0.0002 * dis) {
      scan_neighbor_picked_[i - start_idx] = 1;
    }
  }
}

void LoamFeatureExtractor::SetRegionBuffersFor(size_t start_idx,
                                               size_t end_idx) {
  // resize buffers
  size_t region_size = end_idx - start_idx + 1;
  region_curvature_.resize(region_size);
  region_sort_indices_.resize(region_size);
  region_label_.assign(region_size, PointLabel::SURFACE_LESS_FLAT);

  // calculate point curvatures and reset sort indices
  float point_weight = -2 * params_->curvature_region;

  for (size_t i = start_idx, regionIdx = 0; i <= end_idx; i++, regionIdx++) {
    float diffX = point_weight * sorted_scan_[i].x;
    float diffY = point_weight * sorted_scan_[i].y;
    float diffZ = point_weight * sorted_scan_[i].z;

    for (int j = 1; j <= params_->curvature_region; j++) {
      diffX += sorted_scan_[i + j].x + sorted_scan_[i - j].x;
      diffY += sorted_scan_[i + j].y + sorted_scan_[i - j].y;
      diffZ += sorted_scan_[i + j].z + sorted_scan_[i - j].z;
    }

    region_curvature_[regionIdx] =
        diffX * diffX + diffY * diffY + diffZ * diffZ;
    region_sort_indices_[regionIdx] = i;
  }

  // sort point curvatures
  for (size_t i = 1; i < region_size; i++) {
    for (size_t j = i; j >= 1; j--) {
      if (region_curvature_[region_sort_indices_[j] - start_idx] <
          region_curvature_[region_sort_indices_[j - 1] - start_idx]) {
        std::swap(region_sort_indices_[j], region_sort_indices_[j - 1]);
      }
    }
  }
}

void LoamFeatureExtractor::MarkAsPicked(size_t cloud_idx, size_t scan_idx) {
  scan_neighbor_picked_[scan_idx] = 1;

  for (int i = 1; i <= params_->curvature_region; i++) {
    if (beam::SquaredDiff<pcl::PointXYZ>(sorted_scan_[cloud_idx + i],
                                         sorted_scan_[cloud_idx + i - 1]) >
        0.05) {
      break;
    }
    scan_neighbor_picked_[scan_idx + i] = 1;
  }

  for (int i = 1; i <= params_->curvature_region; i++) {
    if (beam::SquaredDiff<pcl::PointXYZ>(sorted_scan_[cloud_idx - i],
                                         sorted_scan_[cloud_idx - i + 1]) >
        0.05) {
      break;
    }
    scan_neighbor_picked_[scan_idx - i] = 1;
  }
}

} // namespace beam_matching
