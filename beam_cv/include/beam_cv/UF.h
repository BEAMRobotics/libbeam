/** @file
 * @ingroup cv
 */

#pragma once

#include <vector>

using namespace std;

namespace beam_cv {

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

  void Initialize(int n);

  int FindSet(int p);

  void UnionSets(int p, int q);

protected:
  vector<int> id_;
  vector<int> rank_;
};
} // namespace beam_cv