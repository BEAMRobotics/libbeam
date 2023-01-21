/** @file
 * @ingroup utils
 *
 * kdtree: wrapper around nanoflann kdtree to match pcl's kdtree interface
 *
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <beam_utils/nanoflann.hpp>

namespace beam {
/** @addtogroup utils
 *  @{ */

namespace nanoflann {

template <typename T>
struct PointCloud {
  struct Point {
    T x, y, z;
  };

  using coord_t = T; //!< The type of each coordinate

  std::vector<Point> pts;

  // Must return the number of data points
  inline size_t kdtree_get_point_count() const { return pts.size(); }

  // Returns the dim'th component of the idx'th point in the class:
  // Since this is inlined and the "dim" argument is typically an immediate
  // value, the
  //  "if/else's" are actually solved at compile time.
  inline T kdtree_get_pt(const size_t idx, const size_t dim) const {
    if (dim == 0)
      return pts[idx].x;
    else if (dim == 1)
      return pts[idx].y;
    else
      return pts[idx].z;
  }

  // Optional bounding-box computation: return false to default to a standard
  // bbox computation loop.
  //   Return true if the BBOX was already computed by the class and returned
  //   in "bb" so it can be avoided to redo it again. Look at bb.size() to
  //   find out the expected dimensionality (e.g. 2 or 3 for point clouds)
  template <class BBOX>
  bool kdtree_get_bbox(BBOX& /* bb */) const {
    return false;
  }
};

} // namespace nanoflann

const int k_pointcloud_dims{3};
const int k_max_leaf{10};

using KdTreeType = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, nanoflann::PointCloud<float>>,
    nanoflann::PointCloud<float>, k_pointcloud_dims>;

template <class PointT>
class KdTree {
public:
  KdTree(const pcl::PointCloud<PointT>& cloud_in) {
    for (const auto& p : cloud_in) {
      cloud.pts.push_back(
          nanoflann::PointCloud<float>::Point{.x = p.x, .y = p.y, .z = p.z});
    }
    kdtree = std::make_unique<KdTreeType>(k_pointcloud_dims, cloud, k_max_leaf);
  }

  int nearestKSearch(const PointT& p, int k, std::vector<uint32_t>& point_ids,
                     std::vector<float>& point_distances) {
    point_ids = std::vector<uint32_t>(k);
    point_distances = std::vector<float>(k);
    std::vector<float> point_distances_sqr(k);
    const float query_pt[3] = {p.x, p.y, p.z};
    int num_results = kdtree->knnSearch(&query_pt[0], static_cast<size_t>(k),
                                        &point_ids[0], &point_distances_sqr[0]);
    for (int i = 0; i < k; i++) {
      if (point_distances_sqr.at(i) > 0) {
        point_distances.at(i) = std ::sqrt(point_distances_sqr.at(i));
      }
    }
    return num_results;
  }

  size_t radiusSearch(const PointT& p, const float radius,
                      std::vector<int>& point_ids,
                      std::vector<float>& point_distances) {
    point_ids.clear();
    point_distances.clear();
    std::vector<std::pair<uint32_t, float>> ret_matches;
    nanoflann::SearchParams params;
    const float query_pt[3] = {p.x, p.y, p.z};
    size_t n_matches =
        kdtree->radiusSearch(&query_pt[0], radius, ret_matches, params);
    for (size_t i = 0; i < n_matches; i++) {
      point_ids.push_back(ret_matches[i].first);
      point_ids.push_back(ret_matches[i].second);
    }
    return n_matches;
  }

  void setInputCloud(const pcl::PointCloud<PointT>& point_cloud) {
    cloud.pts.clear();
    for (const auto& p : point_cloud) {
      cloud.pts.push_back(
          nanoflann::PointCloud<float>::Point{.x = p.x, .y = p.y, .z = p.z});
    }
    kdtree = std::make_unique<KdTreeType>(k_pointcloud_dims, cloud, k_max_leaf);
  }

  void clear() {
    cloud.pts.clear();
    kdtree = std::make_unique<KdTreeType>(k_pointcloud_dims, cloud, k_max_leaf);
  }
  nanoflann::PointCloud<float> cloud;
  std::unique_ptr<KdTreeType> kdtree;
};

/** @} group utils */
} // namespace beam
