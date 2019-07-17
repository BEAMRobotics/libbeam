/** @file
 * @ingroup defects
 */

#pragma once
#include "beam_defects/Defect.h"

namespace beam_defects {
/** @addtogroup defects
 *  @{ */

/**
 * @brief Derived class for corrosion defects
 */
class Corrosion : public Defect {
public:
  /**
   * @brief Default constructor
   */
  Corrosion() = default;

  /**
   * @brief Construct with a point cloud
   * @param pc
   */
  explicit Corrosion(pcl::PointCloud<pcl::PointXYZ>::Ptr pc);

  /**
   * @brief Default constructor
   */
  ~Corrosion() override = default;

  /**
   * @brief Get the type of defect
   * @return Returns type as one of defects specified in the enum DefectType
   */
  DefectType GetType() const override { return DefectType::CORROSION; };

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

  double corrosion_size_ = 0; // Variable to store corrosion size

};

/** @} group defects */

} // namespace beam_defects
