/** @file
 * @ingroup defects
 * Includes all defects classes / functions
 *
 * @defgroup defects
 * Defect functions
 */

#pragma once

#include <boost/smart_ptr.hpp>
#include <iostream>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace beam_defects {
/** @addtogroup defects
 *  @{ */

/**
 * @brief Enum class for different types of defects we might want to use
 */
enum class DefectType { CRACK = 0, SPALL, DELAM };

/**
 * @brief Enum class for defect severity based on OSIM guidelines
 */
enum class DefectOSIMSeverity { NONE = 0, LIGHT, MEDIUM, SEVERE, VERY_SEVERE };

/**
 * @brief Abstract class for defects
 */
class Defect {
public:
  /**
   * @brief Default constructor
   */
  Defect() = default;

  /**
   * @brief Default destructor
   */
  virtual ~Defect() = default;

  /**
   * @brief Pure virtual method for returning the size of a defect
   * @return Returns size of defect
   */
  virtual double GetSize() = 0;

  /**
   * @brief Pure virtual method for returning type of defect
   * @return
   */
  virtual DefectType GetType() const = 0;

  virtual DefectOSIMSeverity GetOSIMSeverity() = 0;

  using Ptr = std::shared_ptr<Defect>;
};

/** @} group defects */

} // namespace beam_defects
