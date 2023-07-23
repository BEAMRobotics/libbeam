/** @file
 * @ingroup matching
 *
 * @defgroup matching
 * Classes to perform scan matching/registration.
 * There are a variety of different scan matching algorithms
 * available with a similar interface.
 */

#pragma once

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>
#include <pcl/common/transforms.h>

#include <beam_utils/log.h>
#include <beam_utils/pointclouds.h>

namespace beam_matching {
/** @addtogroup matching
 *  @{ */

/**
 * @brief Enum class for different types of matchers
 */
enum class MatcherType { ICP = 0, GICP, NDT, LOAM, UNKNOWN };

/**
 * @brief lookup matcher type from config file. This will load a json and look
 * for matcher_type field
 */
static MatcherType GetTypeFromConfig(const std::string& file_location) {
  if (!boost::filesystem::exists(file_location)) {
    BEAM_ERROR("invalid file path for matcher config, file path: {}",
               file_location);
    return MatcherType::UNKNOWN;
  }

  nlohmann::json J;
  std::ifstream file(file_location);
  file >> J;

  if (!J.contains("matcher_type")) {
    BEAM_ERROR("invalid matcher config file, no matcher_type field.");
    return MatcherType::UNKNOWN;
  }

  std::string matcher_type = J["matcher_type"];

  if (matcher_type == "ICP") {
    return MatcherType::ICP;
  } else if (matcher_type == "GICP") {
    return MatcherType::GICP;
  } else if (matcher_type == "NDT") {
    return MatcherType::NDT;
  } else if (matcher_type == "LOAM") {
    return MatcherType::LOAM;
  } else {
    BEAM_ERROR("invalid matcher_type field");
    return MatcherType::UNKNOWN;
  }
}

/**
 * This class is inherited by each of the different matching algorithms.
 * @tparam T sensor data type
 */
template <typename T>
class Matcher {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * @brief This constructor takes an argument in order to adjust how much
   * downsampling is done before matching is attempted. Pointclouds are
   * downsampled using a voxel filter.
   * @param res the edge length of each voxel.
   */
  Matcher(float res) : resolution_(res) {}

  /**
   * @brief Default constructor. Sets up the instance of the matcher to attempt
   * to match using the full resolution of each point cloud (if possible).
   */
  Matcher() { resolution_ = -1; }

  virtual ~Matcher() {}

  /**
   * @brief Returns T_TARGET_REF
   */
  Eigen::Affine3d GetResult() { return this->result_; };

  /**
   * @brief returns the calculated covariance matrix.
   * Covariance has the form: 6x6 matrix [dx, dy, dz, dqx, dqy, dqz]
   */
  Eigen::Matrix<double, 6, 6>& GetCovariance() {
    CalculateCovariance();
    return this->covariance_;
  };

  float GetRes() { return this->resolution_; };

  /**
   * @brief `setRef` and `setTarget` are implemented for each specific matching
   * algorithm and are how the pointclouds are passed to the matching object.
   * Note that there isn't a parameter for an initial transform; the initial
   * transform is always assumed to be identity inside the matcher class. If
   * an initial transform estimate is available, the target pointcloud should
   * be transformed by it before being passed to the class as some algorithms
   * require a good initial estimate to perform well.
   */
  virtual void SetRef(const T& ref) = 0;
  virtual void SetTarget(const T& target) = 0;
  void Setup(const T& ref, const T& target) {
    this->SetRef(ref);
    this->SetTarget(target);
  };

  /**
   * @brief Actually performs the match. Any heavy processing is done here.
   * @returns true if match was successful, false if match was not successful
   */
  virtual bool Match() = 0;

  /**
   * @brief Pure virtual function for saving results. Stores the results as 3
   * separate clouds:
   *
   *  (1) reference cloud (blue points)
   *
   *  (2) target cloud initial: target cloud transformed into the reference
   *  cloud frame using the initial guess of it's relative pose. Since this
   *  matcher class doesn't allow you to provide an initial guess, the target
   *  cloud should already be in the reference frame using some guess.
   *
   *  (3) target cloud refined: target cloud transformed into the reference
   *  cloud frame using the refined pose (or transform calculated herein)
   *
   */
  virtual void SaveResults(const std::string& output_dir,
                           const std::string& prefix = "cloud") = 0;

protected:
  virtual void CalculateCovariance() = 0;

  /**
   * @brief for pcl::PointCloud<pcl::PointXYZ> (which is most of the case for
   * matchers) this function can be called as an implementation to the above
   * SaveResults()
   */
  inline void SaveResultsPCLXYZ(const std::string& output_dir,
                                const std::string& prefix,
                                const PointCloudPtr& ref,
                                const PointCloudPtr& target) {
    // check dir exits
    if (!boost::filesystem::exists(output_dir)) {
      BEAM_WARN(
          "Output path does not exist, cannot save matcher results. Input: {}",
          output_dir);
      return;
    }
    std::string output_path = beam::CombinePaths(output_dir, prefix);

    // transform tgt cloud
    PointCloud tgt_aligned;
    Eigen::Matrix4d T_REF_TGT = result_.matrix();
    pcl::transformPointCloud(*target, tgt_aligned, T_REF_TGT);

    // color pointclouds and add frames
    PointCloudCol ref_col = beam::ColorPointCloud(*ref, 0, 0, 255);
    PointCloudCol tgt_init_col = beam::ColorPointCloud(*ref, 255, 0, 0);
    PointCloudCol tgt_align_col = beam::ColorPointCloud(tgt_aligned, 0, 255, 0);

    // add frames to clouds
    PointCloudCol frame = beam::CreateFrameCol();
    beam::MergeFrameToCloud(ref_col, frame, Eigen::Matrix4d::Identity());
    beam::MergeFrameToCloud(tgt_align_col, frame, Eigen::Matrix4d::Identity());
    beam::MergeFrameToCloud(tgt_align_col, frame, T_REF_TGT);

    std::string error_type;
    if (!beam::SavePointCloud<pcl::PointXYZRGB>(
            output_path + "reference.pcd", ref_col,
            beam::PointCloudFileType::PCDBINARY, error_type)) {
      BEAM_WARN("Cannot save reference scan. Reason: {}", error_type);
    }

    if (!beam::SavePointCloud<pcl::PointXYZRGB>(
            output_path + "target_initial.pcd", tgt_init_col,
            beam::PointCloudFileType::PCDBINARY, error_type)) {
      BEAM_WARN("Cannot save initial target scan. Reason: {}", error_type);
    }

    if (!beam::SavePointCloud<pcl::PointXYZRGB>(
            output_path + "target_aligned.pcd", tgt_align_col,
            beam::PointCloudFileType::PCDBINARY, error_type)) {
      BEAM_WARN("Cannot save aligned target scan. Reason: {}", error_type);
    }
  }

  /**
   * The edge length (in the same distance-units as those used in the
   * pointcloud) of a downsampling voxel filter. If no downsampling is
   * happening, this will be -1
   */
  float resolution_;

  /**
   * The transformation calculated by the scan registration algorithm. It is
   * the transformation needed to transform the target pointcloud to the
   * reference pointcloud in the reference frame of the reference pointcloud
   * (T_REF_TGT).
   */
  Eigen::Affine3d result_;

  /**
   * The information matrix calculated by the scan-matching algorithm. (6x6
   * matrix: dx, dy, dz, dqx, dqy, dqz)
   */
  Eigen::Matrix<double, 6, 6> covariance_{
      Eigen::Matrix<double, 6, 6>::Identity()};
};

/** @} group matching */
} // namespace beam_matching
