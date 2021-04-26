/** @file
 * @ingroup points  
 * Includes all custom pcl point types
 *
 * @ defgroup points
 * Custom Beam pcl point types
 */

#pragma once

#include <pcl/point_types.h>

namespace beam_containers {
/** @addtogroup points
 *  @{ */

/**
 * @brief Point type for storing data to use for bridge inspection
 *
 * The fields used for bridge inspection are:
 * XYZ+Padding, RGB, laser intensity, grayscale thermal,
 * crack probaility, spall probability, corrosion probability,
 * and delamination probability. 
 */

struct PointBridge {
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  float intensity;
  PCL_ADD_RGB;                      // preferred way of adding RGB channels                  
  uint8_t thermal;                  
  float crack;                      
  float spall;                      
  float corrosion;                  
  float delam;                      
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // ensure new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

/** @} group points */

}; // namespace beam_containers

POINT_CLOUD_REGISTER_POINT_STRUCT (beam_containers::PointBridge,           
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (float, rgb, rgb)
                                   (std::uint8_t, thermal, thermal)
                                   (float, crack, crack)
                                   (float, spall, spall)
                                   (float, corrosion, corrosion)
                                   (float, delam, delam)
)
