/** @file
 * @ingroup utils
 */

#pragma once

#include <vector>

using namespace std;

namespace beam {

/**
 * @brief This class provides a simple yet efficient Union-Find data structure
 * which is helpful in finding disjoint sets in various datasets:
 * https://en.wikipedia.org/wiki/Disjoint-set_data_structure
 * Used in finding connected components in single channel images since opencv's
 * connected components algorithm is only implemented for binary images
 */
class UnionFind {
public:
  /**
   * @brief Default constructor
   */
  UnionFind() = default;

  /**
   * @brief Default destructor
   */
  ~UnionFind() = default;

  /**
   * @brief Creates UnionFind structure with n items
   */
  void Initialize(int n);

  /**
   * @brief Returns parent of set containing p
   */
  int FindSet(int p);

  /**
   * @brief Performs set union on sets containing p and q
   */
  void UnionSets(int p, int q);

protected:
  vector<int> id_;
  vector<int> rank_;
};
} // namespace beam