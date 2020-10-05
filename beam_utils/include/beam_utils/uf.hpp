/** @file
 * @ingroup utils
 */

#pragma once

#include <vector>

using namespace std;

namespace beam {

/**
 * @brief This class provides a simple yet efficient Union-Find data structure
 * which is helpful in finding disjoint sets in various datasets
 */
class UF {
public:
  /**
   * @brief Default constructor
   */
  UF() = default;

  /**
   * @brief Default destructor
   */
  ~UF() = default;

  /**
   * @brief Creates UF structure with n items
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