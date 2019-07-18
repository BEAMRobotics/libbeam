/** @file
 * @ingroup defects
 * Includes all defects classes / functions
 *
 * @defgroup defects
 * Defect functions
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/smart_ptr.hpp>
#include <memory>

namespace beam_defects {
/** @addtogroup defects
 *  @{ */

/**
 * @brief Enum class for different types of defects we might want to use
 */
enum class DefectType { CRACK = 0, SPALL, DELAM, CORROSION };

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
   * @brief Construct with a point cloud
   * @param pc
   */
  explicit Defect(pcl::PointCloud<pcl::PointXYZ>::Ptr pc);

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
   * @brief Method for returning the height and width dimensions of a 2D
   * bounding box around the defect after projection into a 2D plane
   * @return Returns a vector [delta x, delta y] representing the
   * bounding box
   */
  std::vector<float> GetBBoxDims2D();

  float GetMaxDim2D();

  /**
   * @brief Pure virtual method for returning type of defect
   * @return
   */
  virtual DefectType GetType() const = 0;

  virtual DefectOSIMSeverity GetOSIMSeverity() = 0;

  using Ptr = std::shared_ptr<Defect>;

protected:
  // Variable for storing the defect point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr defect_cloud_ =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr defect_cloud_hull_ =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  pcl::PointCloud<pcl::PointXYZ>::Ptr CalculateHull2D();

  bool defect_cloud_initialized_;
};

/** @} group defects */

} // namespace beam_defects
