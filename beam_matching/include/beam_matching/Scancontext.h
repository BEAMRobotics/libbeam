#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <beam_utils/nanoflann.hpp>
#include <beam_utils/time.h>

// ===== This example shows how to use nanoflann with these types of containers:
// =======
// typedef std::vector<std::vector<double> > my_vector_of_vectors_t;
// typedef std::vector<Eigen::VectorXd> my_vector_of_vectors_t;   // This
// requires #include <Eigen/Dense>
// =====================================================================================

/** A simple vector-of-vectors adaptor for nanoflann, without duplicating the
 * storage. The i'th vector represents a point in the state space.
 *
 *  \tparam DIM If set to >0, it specifies a compile-time fixed dimensionality
 * for the points in the data set, allowing more compiler optimizations. \tparam
 * num_t The type of the point coordinates (typically, double or float). \tparam
 * Distance The distance metric to use: beam::nanoflann::metric_L1,
 * beam::nanoflann::metric_L2, beam::nanoflann::metric_L2_Simple, etc. \tparam
 * IndexType The type for indices in the KD-tree index (typically, size_t of
 * int)
 */
template <class VectorOfVectorsType, typename num_t = double, int DIM = -1,
          class Distance = beam::nanoflann::metric_L2,
          typename IndexType = size_t>
struct KDTreeVectorOfVectorsAdaptor {
  typedef KDTreeVectorOfVectorsAdaptor<VectorOfVectorsType, num_t, DIM,
                                       Distance>
      self_t;
  typedef
      typename Distance::template traits<num_t, self_t>::distance_t metric_t;
  typedef beam::nanoflann::KDTreeSingleIndexAdaptor<metric_t, self_t, DIM,
                                                    IndexType>
      index_t;

  index_t* index; //! The kd-tree index for the user to call its methods as
                  //! usual with any other FLANN index.

  /// Constructor: takes a const ref to the vector of vectors object with the
  /// data points
  KDTreeVectorOfVectorsAdaptor(const size_t /* dimensionality */,
                               const VectorOfVectorsType& mat,
                               const int leaf_max_size = 10)
      : m_data(mat) {
    assert(mat.size() != 0 && mat[0].size() != 0);
    const size_t dims = mat[0].size();
    if (DIM > 0 && static_cast<int>(dims) != DIM)
      throw std::runtime_error(
          "Data set dimensionality does not match the 'DIM' template argument");
    index = new index_t(
        static_cast<int>(dims), *this /* adaptor */,
        beam::nanoflann::KDTreeSingleIndexAdaptorParams(leaf_max_size));
    index->buildIndex();
  }

  ~KDTreeVectorOfVectorsAdaptor() { delete index; }

  const VectorOfVectorsType& m_data;

  /** Query for the \a num_closest closest points to a given point (entered as
   * query_point[0:dim-1]). Note that this is a short-cut method for
   * index->findNeighbors(). The user can also call index->... methods as
   * desired. \note nChecks_IGNORED is ignored but kept for compatibility with
   * the original FLANN interface.
   */
  inline void query(const num_t* query_point, const size_t num_closest,
                    IndexType* out_indices, num_t* out_distances_sq,
                    const int nChecks_IGNORED = 10) const {
    beam::nanoflann::KNNResultSet<num_t, IndexType> resultSet(num_closest);
    resultSet.init(out_indices, out_distances_sq);
    index->findNeighbors(resultSet, query_point,
                         beam::nanoflann::SearchParams());
  }

  /** @name Interface expected by KDTreeSingleIndexAdaptor
   * @{ */

  const self_t& derived() const { return *this; }
  self_t& derived() { return *this; }

  // Must return the number of data points
  inline size_t kdtree_get_point_count() const { return m_data.size(); }

  // Returns the dim'th component of the idx'th point in the class:
  inline num_t kdtree_get_pt(const size_t idx, const size_t dim) const {
    return m_data[idx][dim];
  }

  // Optional bounding-box computation: return false to default to a standard
  // bbox computation loop.
  //   Return true if the BBOX was already computed by the class and returned in
  //   "bb" so it can be avoided to redo it again. Look at bb.size() to find out
  //   the expected dimensionality (e.g. 2 or 3 for point clouds)
  template <class BBOX>
  bool kdtree_get_bbox(BBOX& /*bb*/) const {
    return false;
  }

  /** @} */

}; // end of KDTreeVectorOfVectorsAdaptor

using namespace Eigen;

using std::cout;
using std::endl;
using std::make_pair;

using std::atan2;
using std::cos;
using std::sin;

using SCPointType =
    pcl::PointXYZI; // using xyz only. but a user can exchange the original bin
                    // encoding function (i.e., max hegiht) to max intensity
                    // (for detail, refer 20 ICRA Intensity Scan Context)
using KeyMat = std::vector<std::vector<float> >;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor<KeyMat, float>;

// namespace SC2
// {

void coreImportTest(void);

// sc param-independent helper functions
float xy2theta(const float& _x, const float& _y);
MatrixXd circshift(MatrixXd& _mat, int _num_shift);
std::vector<float> eig2stdvec(MatrixXd _eigmat);

class SCManager {
public:
  SCManager() =
      default; // reserving data space (of std::vector) could be considered. but
               // the descriptor is lightweight so don't care.

  Eigen::MatrixXd makeScancontext(pcl::PointCloud<SCPointType>& _scan_down);
  Eigen::MatrixXd makeRingkeyFromScancontext(Eigen::MatrixXd& _desc);
  Eigen::MatrixXd makeSectorkeyFromScancontext(Eigen::MatrixXd& _desc);

  int fastAlignUsingVkey(MatrixXd& _vkey1, MatrixXd& _vkey2);
  double distDirectSC(
      MatrixXd& _sc1,
      MatrixXd& _sc2); // "d" (eq 5) in the original paper (IROS 18)
  std::pair<double, int> distanceBtnScanContext(
      MatrixXd& _sc1,
      MatrixXd& _sc2); // "D" (eq 6) in the original paper (IROS 18)

  // User-side API
  void makeAndSaveScancontextAndKeys(pcl::PointCloud<SCPointType>& _scan_down);
  std::pair<int, float>
      detectLoopClosureID(void); // int: nearest node index, float: relative yaw

public:
  // hyper parameters ()
  const double LIDAR_HEIGHT =
      2.0; // lidar height : add this for simply directly using lidar scan in
           // the lidar local coord (not robot base coord) / if you use
           // robot-coord-transformed lidar scans, just set this as 0.

  const int PC_NUM_RING = 20;   // 20 in the original paper (IROS 18)
  const int PC_NUM_SECTOR = 60; // 60 in the original paper (IROS 18)
  const double PC_MAX_RADIUS =
      80.0; // 80 meter max in the original paper (IROS 18)
  const double PC_UNIT_SECTORANGLE = 360.0 / double(PC_NUM_SECTOR);
  const double PC_UNIT_RINGGAP = PC_MAX_RADIUS / double(PC_NUM_RING);

  // tree
  const int NUM_EXCLUDE_RECENT = 50; // simply just keyframe gap, but node
                                     // position distance-based exclusion is ok.
  const int NUM_CANDIDATES_FROM_TREE =
      10; // 10 is enough. (refer the IROS 18 paper)

  // loop thres
  const double SEARCH_RATIO =
      0.1; // for fast comparison, no Brute-force, but search 10 % is okay. //
           // not was in the original conf paper, but improved ver.
  const double SC_DIST_THRES =
      0.13; // empirically 0.1-0.2 is fine (rare false-alarms) for 20x60 polar
            // context (but for 0.15 <, DCS or ICP fit score check (e.g., in
            // LeGO-LOAM) should be required for robustness)
  // const double SC_DIST_THRES = 0.5; // 0.4-0.6 is good choice for using with
  // robust kernel (e.g., Cauchy, DCS) + icp fitness threshold / if not,
  // recommend 0.1-0.15

  // config
  const int TREE_MAKING_PERIOD_ =
      50; // i.e., remaking tree frequency, to avoid non-mandatory every
          // remaking, to save time cost / if you want to find a very recent
          // revisits use small value of it (it is enough fast ~ 5-50ms wrt N.).
  int tree_making_period_conter = 0;

  // data
  std::vector<double> polarcontexts_timestamp_; // optional.
  std::vector<Eigen::MatrixXd> polarcontexts_;
  std::vector<Eigen::MatrixXd> polarcontext_invkeys_;
  std::vector<Eigen::MatrixXd> polarcontext_vkeys_;

  KeyMat polarcontext_invkeys_mat_;
  KeyMat polarcontext_invkeys_to_search_;
  std::unique_ptr<InvKeyTree> polarcontext_tree_;

}; // SCManager

// } // namespace SC2
