/** @file
 * @ingroup matching
 *
 * @defgroup matching
 * Classes to perform scan matching/registration.
 * There are a variety of different scan matching algorithms
 * available with a similar interface.
 */

#pragma once

#include <beam_utils/utils.h>

namespace beam_matching {
/** @addtogroup matching
 *  @{ */

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
   * @brief Default constructor. Sets up the instance of the matcher to attempt to
   * match using the full resolution of each point cloud (if possible).
   */
  Matcher() { resolution_ = -1; }

  virtual ~Matcher() {}

  Eigen::Affine3d GetResult() { return this->result_; };

  /**
   * @brief returns the calculated information matrix. Make sure you call
   * EstimateInfo() before
   */
  Eigen::Matrix<double, 6, 6>& GetInfo() { return this->information_; };

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

  /** @brief Actually performs the match. Any heavy processing is done here.
   * @returns true if match was successful, false if match was not successful
   */
  virtual bool Match() { return 0; }

  virtual void EstimateInfo() {
    this->information_ = Eigen::Matrix<double, 6, 6>::Identity(6, 6);
  }

protected:
  /**
   * The edge length (in the same distance-units as those used in the
   * pointcloud) of a downsampling voxel filter. If no downsampling is
   * happening, this will be -1
   */
  float resolution_;

  /**
   * The transformation calculated by the scan registration algorithm. It is
   * the transformation needed to transform the target pointcloud to the
   * reference pointcloud in the reference frame of the reference pointcloud.
   */
  Eigen::Affine3d result_;

  /**
   * The information matrix calculated by the scan-matching algorithm. The
   * diagonal elements correpond to translational perturbations in the x, y,
   * and z directions and angular perturbations on the 3-2-1 euler angles, in
   * that order and in the frame of the reference pointcloud. This may change
   * as the kinematics module of libbeam progresses.
   */
  Eigen::Matrix<double, 6, 6> information_;
};

/** @} group matching */
} // namespace beam_matching
