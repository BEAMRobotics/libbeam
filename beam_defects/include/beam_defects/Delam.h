/** @file
 * @ingroup defects
 */

#pragma once
#include "beam_defects/Defect.h"

namespace beam_defects {
/** @addtogroup defects
 *  @{ */

/**
 * @brief Derived class for delamination defects
 */
class Delam : public Defect {
public:
  // Inherit base class constructors
  using Defect::Defect;

  /**
   * @brief Default destructor
   */
  ~Delam() override = default;

  /**
   * @brief Get the type of defect
   * @return Returns type as one of defects specified in the enum DefectType
   */
  DefectType GetType() const override { return DefectType::DELAM; };

  /**
   * @brief Get the size of the defect
   * @return Returns the size of the defect in whatever units are relevant for
   * this class of defect
   */
  double GetSize() override;

  DefectOSIMSeverity GetOSIMSeverity() override;

private:
  /**
   * @brief Method for calculating the size of the defect
   * @return Returns the size
   */
  double CalculateSize();

  double delam_size_ = 0; // Variable to store delam size

};

/** @} group defects */

} // namespace beam_defects
