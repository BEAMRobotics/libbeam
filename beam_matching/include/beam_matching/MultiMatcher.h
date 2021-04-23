/** @file
 * @ingroup matching
 *
 * @defgroup multi_matcher
 * Class to hold multiple instances of a matcher and multithread matches
 */

#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <tuple>

#include <beam_matching/Matcher.h>
#include <beam_utils/math.h>
#include <beam_utils/pointclouds.h>

namespace beam_matching {
/** @addtogroup matching
 *  @{ */

/**
 * @brief Class is templated for different matcher types
 * @tparam T matcher type
 * @tparam R matcher params type
 */
template <typename T, typename R>
class MultiMatcher {
public:
  MultiMatcher(int n_threads = std::thread::hardware_concurrency(),
               int queue_s = 10, R params = R())
      : n_thread(n_threads), queue_size(queue_s), config(params) {
    this->stop = false;
    this->remaining_matches = 0;
    this->InitPool(params);
  }

  ~MultiMatcher();

  /**
   * @brief inserts a pair of scans into the queue to be matched. The resulting
   * transform is the transform used to map
   * the source pointcloud to the target pointcloud
   * @param id for result
   * @param source pointcloud
   * @param target pointcloud
   */
  void Insert(const int& id, const PointCloudPtr& src,
              const PointCloudPtr& target);

  /**
   * @brief Checks to see if there are any remaining matches in the queue.
   * @return bool
   */
  bool Done();

  /**
   * @brief Gets a result at the start of the queue. Will block until a result
   * is ready if the output buffer is empty but there are matches pending.
   * @param id id of result
   * @param transform result
   * @param info information matrix of match
   * @return true if result has been output, false if the output queue is
   * empty and there are no matches pending
   */
  bool GetResult(int* id, Eigen::Affine3d* transform,
                 Eigen::Matrix<double, 6, 6>* info);

private:
  /**
   * @brief Function run by each worker thread
   * @param threadid pair of clouds to match
   */
  void Spin(int threadid);
  void InitPool(R params);

  const int n_thread;
  const int queue_size;
  int remaining_matches;
  R config;
  std::queue<std::tuple<int, PointCloudPtr, PointCloudPtr>> input;
  std::queue<std::tuple<int, Eigen::Affine3d, Eigen::Matrix<double, 6, 6>>>
      output;
  std::vector<std::thread> pool;
  std::vector<T, Eigen::aligned_allocator<T>> matchers;

  // Synchronization
  std::mutex ip_mutex;
  std::mutex op_mutex;
  std::mutex cnt_mutex;
  std::condition_variable ip_condition;
  std::condition_variable op_condition;
  bool stop;
};

} // namespace beam_matching

#include <beam_matching/MultiMatcherImpl.hpp>
