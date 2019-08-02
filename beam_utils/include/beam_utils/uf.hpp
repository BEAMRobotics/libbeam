/** @file
 * @ingroup cv
 */

#pragma once

#include <vector>

using namespace std;

namespace beam {

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