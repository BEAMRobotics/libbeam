/** @file
 * @ingroup filtering
 */

#pragma once

// beam
#include "beam_utils/math.hpp"

namespace beam_filtering {
/** @addtogroup filtering
 *  @{ */

/**
 * @brief class for crop box filter
 */
class DROR {
public:
  /**
   * @brief Default constructor
   */
  DROR() = default;

  /**
   * @brief Default destructor
   */
  ~DROR() = default;

  /**
   * @brief Method for retrieving Var
   * @return Var
   */
  double GetVar();

  /**
   * @brief Method for setting var
   * @param var
   */
  void SetVar(double &var);


private:
  double var_;
};

/** @} group filtering */

} // namespace beam_filtering
