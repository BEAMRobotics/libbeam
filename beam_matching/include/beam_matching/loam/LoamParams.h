/** @file
 * @ingroup matching
 *
 * The is an implementation of Lidar Odometry and Mapping (LOAM). See the
 * following papers:
 *
 *    Zhang, J., & Singh, S. (2014). LOAM : Lidar Odometry and Mapping in
 * Real-time. Robotics: Science and Systems.
 * https://doi.org/10.1007/s10514-016-9548-2
 *
 *    Zhang, J., & Singh, S. (2018). Laser–visual–inertial odometry and mapping
 * with high robustness and low drift. Journal of Field Robotics, 35(8),
 * 1242–1264. https://doi.org/10.1002/rob.21809
 *
 * This code was derived from the following repos:
 *
 *    https://github.com/laboshinl/loam_velodyne
 *
 *    https://github.com/libing64/lidar_pose_estimator
 *
 *    https://github.com/TixiaoShan/LIO-SAM
 *
 */

#pragma once

#include <vector>

#include <boost/filesystem.hpp>

#include <beam_optimization/CeresParams.h>
#include <beam_utils/filesystem.h>

namespace beam_matching {
/** @addtogroup matching
 *  @{ */

/**
 * @brief Struct for storing all LOAM params. This will be used by the
 * LoamPointCloud, LoamFeatureExtractor, LoamScanRegistration and LoamMatcher.
 * therefore it is normally recommended to use a shared pointer to this obeject.
 */
class LoamParams {
public:
  /**
   * @brief default constructor
   */
  LoamParams() = default;

  /**
   * @brief construct given json config file
   * @param param_config full path to json config file
   */
  LoamParams(const std::string& param_config, std::string ceres_config = "") {
    if (param_config.empty()) { return; }

    std::string read_file = param_config;
    if (param_config == "DEFAULT_PATH") {
      read_file = beam::LibbeamRoot();
      read_file += "beam_matching/config/loam.json";
    }

    BEAM_INFO("Loading LOAM config file: {}", read_file);
    nlohmann::json J;
    if (!beam::ReadJson(read_file, J)) {
      BEAM_ERROR("Using default loam params.");
      return;
    }

    beam::ValidateJsonKeysOrThrow(
        std::vector<std::string>{"number_of_beams",
                                 "fov_deg",
                                 "n_feature_regions",
                                 "curvature_region",
                                 "max_corner_sharp",
                                 "max_surface_flat",
                                 "max_corner_less_sharp",
                                 "max_surface_flat",
                                 "less_flat_filter_size",
                                 "surface_curvature_threshold",
                                 "vertical_axis",
                                 "max_correspondence_distance",
                                 "validate_correspondences",
                                 "iterate_correspondences",
                                 "min_number_measurements",
                                 "convergence_criteria_translation_m",
                                 "convergence_criteria_rotation_deg",
                                 "max_correspondence_iterations",
                                 "output_ceres_summary",
                                 "output_optimization_summary",
                                 "ceres_config",
                                 "downsample_less_flat_features",
                                 "check_strong_features_first",
                                 "ignore_weak_features"},
        J);

    number_of_beams = J["number_of_beams"];
    fov_deg = J["fov_deg"];
    n_feature_regions = J["n_feature_regions"];
    curvature_region = J["curvature_region"];
    max_corner_sharp = J["max_corner_sharp"];
    max_corner_less_sharp = J["max_corner_less_sharp"];
    max_surface_flat = J["max_surface_flat"];
    less_flat_filter_size = J["less_flat_filter_size"];
    downsample_less_flat_features = J["downsample_less_flat_features"];
    surface_curvature_threshold = J["surface_curvature_threshold"];
    check_strong_features_first = J["check_strong_features_first"];
    ignore_weak_features = J["ignore_weak_features"];
    vertical_axis = J["vertical_axis"];
    max_correspondence_distance = J["max_correspondence_distance"];
    validate_correspondences = J["validate_correspondences"];
    iterate_correspondences = J["iterate_correspondences"];
    min_number_measurements = J["min_number_measurements"];
    convergence_criteria_translation_m =
        J["convergence_criteria_translation_m"];
    convergence_criteria_rotation_deg = J["convergence_criteria_rotation_deg"];
    max_correspondence_iterations = J["max_correspondence_iterations"];
    output_ceres_summary = J["output_ceres_summary"];
    output_optimization_summary = J["output_optimization_summary"];

    if (!check_strong_features_first && ignore_weak_features) {
      BEAM_WARN(
          "If ignore_weak_features is set to true, check_strong_features_first "
          "must be set to true. Changing check_strong_features_first to true.");
      check_strong_features_first = true;
    }
    if (ceres_config.empty()) {
      std::string ceres_config_read = J["ceres_config"];
      optimizer_params = beam_optimization::CeresParams(ceres_config_read);
    } else {
      optimizer_params = beam_optimization::CeresParams(ceres_config);
    }
  }

  /**
   * @brief To separate points into beams (or rings) we need to separate them
   * into bins based on number of beams and FOV. This can be calculated here
   * once instead of each time we get a new scan
   */
  inline std::vector<double> GetBeamAngleBinsDeg() {
    if (beam_angle_bins_.size() != 0) { return beam_angle_bins_; }

    double current_angle = fov_deg / 2 - fov_deg / (number_of_beams - 1) / 2;
    while (current_angle >= -fov_deg / 2) {
      beam_angle_bins_.push_back(current_angle);
      current_angle -= fov_deg / (number_of_beams - 1);
    }
    return beam_angle_bins_;
  }

  /** @brief Used to separate cloud into rings of points if clouds are not
   * provided as pcl::PointXYZL (where the label is the ring number). Point
   * smoothness is calculated by searching neighbors on the same ring. If the
   * cloud is aggregated over multiple scans, then multiple rings of points will
   * be used.*/
  int number_of_beams{16};

  /** @brief Used along with number_of_beams to separate cloud into rings of
   * points if clouds are not provided as pcl::PointXYZL (where the label is the
   * ring number)*/
  double fov_deg{30};

  /** The number of (equally sized) regions used to distribute the feature
   * extraction within a scan. */
  int n_feature_regions{6};

  /** The number of surrounding points (+/- region around a point) used to
   * calculate a point curvature. */
  int curvature_region{5};

  /** The maximum number of sharp corner points per feature region. */
  int max_corner_sharp{2};

  /** The maximum number of less sharp corner points per feature region. */
  int max_corner_less_sharp{10 * 2};

  /** The maximum number of flat surface points per feature region. */
  int max_surface_flat{4};

  /** The curvature threshold below / above a point is considered a flat /
   * corner point. */
  float surface_curvature_threshold{0.1};

  /** Vertical axis of the lidar. Used to separate the cloud ino rings. */
  std::string vertical_axis{"Z"};

  /** Maximum distance to look for correspondences. If min numer of
   * correspondences (2 for edges and 3 for planes) isn't found, and less sharp
   * or less flat features are available, we will use those for the
   * correspondences. */
  double max_correspondence_distance{0.5};

  /** TODO: Validate correspondences by doing an eigen value analysis. This
   * probably requires a denser reference scan to check neighbors*/
  bool validate_correspondences{false};

  /** Iteratively calculate correspondences after each pose solution is
   * estiamted. Set this to true when not confident on the initial estimate. */
  bool iterate_correspondences{true};

  /** Minimum number of measurements (i.e. corresponding features between the
   * two clouds) to perform registration. */
  size_t min_number_measurements{30};

  /** Correspondence iteration will stop if change in translation is below this
   * criteria (and change in rotation is below rotation criteria). In Meters.
   * Note this compares the norm of the translation vector */
  double convergence_criteria_translation_m{0.001};

  /** Correspondence iteration will stop if change in rotation is below this
   * criteria (and change in translation is below translation criteria). In
   * Degrees. Note this compares the angle part of the Axis-Angle rotation
   * representation */
  double convergence_criteria_rotation_deg{1};

  /** Maximum number of times to iterate the correspondences */
  int max_correspondence_iterations{10};

  /** all params needed for ceres */
  beam_optimization::CeresParams optimizer_params;

  /** set to true to cout ceres summary at each iteration */
  bool output_ceres_summary{false};

  /** set to true to cout optimization summaries for each iteration */
  bool output_optimization_summary{false};

  /** If set to true, we look for correspondences in strong features and only go
   * to weak features if we don't have enough. If set to false, we immediately
   * look at weak features which is a combination of strong and weak */
  bool check_strong_features_first{true};

  /** If set to true, we will only extract strong features and use them for
   * registration */
  bool ignore_weak_features{false};

  /** Runs voxel filter on less flat (weak surface) features using voxel size:
   * less_flat_filter_size */
  bool downsample_less_flat_features{false};
  float less_flat_filter_size{0.2};

private:
  std::vector<double> beam_angle_bins_;
};

using LoamParamsPtr = std::shared_ptr<LoamParams>;

/** @} group matching */
} // namespace beam_matching
