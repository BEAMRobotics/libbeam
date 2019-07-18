/** @file
 * @ingroup cv
 */

#pragma once
// beam
#include "beam_utils/math.hpp"

// OpenCV
#include <opencv2/opencv.hpp>

namespace beam_cv {

class Morphology {
public:
  /**
   * @brief Default constructor
   */
  Morphology() = default;

  /**
   * @brief Default destructor
   */
  virtual ~Morphology() = default;

protected:
};

} // namespace beam_cv