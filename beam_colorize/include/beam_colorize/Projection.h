/** @file
 * @ingroup colorizer
 */

#pragma once
#include "beam_colorize/Colorizer.h"

namespace beam_colorize {
/** @addtogroup colorizer
 *  @{ */

/**
 * @brief Class which implements Colorizer interface and provides colorization
 * functionality using projection methods
 */
class Projection : public Colorizer {
public:
  Projection();

  ~Projection() override = default;

  /**
   * @brief see Colorizer.h
   */
  ProjectionMap CreateProjectionMap(
      const PointCloudCol::Ptr& cloud_in_camera_frame) const override;

  /**
   * @brief see Colorizer.h
   */
  ProjectionMap CreateProjectionMap(
      const DefectCloud::Ptr& cloud_in_camera_frame) const override;

private:
};
/** @} group colorizer */

} // namespace beam_colorize
