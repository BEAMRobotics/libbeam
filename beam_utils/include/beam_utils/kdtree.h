/** @file
 * @ingroup utils
 *
 * kdtree: wrapper around nanoflann kdtree to match pcl's kdtree interface
 *
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

const int k_pointcloud_dims{3};
const int k_max_leaf{10};

using KdTreeType = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, PointCloud<float>>, PointCloud<float>,
    k_pointcloud_dims>;

template <class PointT>
class KdTree {
public:
  KdTree(const pcl::Pointcloud<PointT>& cloud_in) {
    for (const auto& p : cloud_in) {
      cloud.pts.push_back(Point{.x = p.x, .y = p.y, z = p.z});
    }
    kdtree = std::make_unique<KdTreeType>(k_pointcloud_dims, cloud, k_max_leaf);
  }

  void nearestKSearch(const PointT& p, std::vector<int>& point_ids,
                      std::vector<float>& point_distances) {
    // todo
  }

  PointCloud<float> cloud;
  std::unique_ptr<KdTreeType> kdtree;
};

} // namespace nanoflann

/** @} group utils */
} // namespace beam
