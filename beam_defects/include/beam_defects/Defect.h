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
enum class DefectOSIMSeverity { LIGHT = 0, MEDIUM, SEVERE, VERY_SEVERE };

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

  // alias for clarity
  using Ptr = std::shared_ptr<Defect>;

  /**
   * @brief Method to set the point cloud attribute defect_cloud_
   * @param pc Pointer to a point cloud
   */
  void SetPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pc);

  pcl::PointCloud<pcl::PointXYZ>::Ptr GetPointCloud();

  /**
   * @brief Method to set the alpha value used when calculating a hull.
   * @param alpha Input value of alpha. If no value is specified, alpha is set
   * to the default value of 0.1
   */
  void SetHullAlpha(float alpha = 0.1);

  /**
   * @brief Pure virtual method for returning the size of a defect
   * @return Returns size of defect
   */
  virtual double GetSize() = 0;

  /**
   * @brief Method that uses icp to get the best matching defect from a vector
   * of defect objects
   * @param defect_vector A vector of pointers to defect objects
   * @return An integer representing the location of where the matching defect
   * is in the input defect vector
   */
  int GetMatchingDefect(std::vector<Defect::Ptr>& defect_vector);

  /**
   * @brief Method for returning the height and width dimensions of a 2D
   * bounding box around the defect after projection into a 2D plane
   * @return Returns a vector [delta x, delta y] representing the
   * bounding box
   */
  std::vector<float> GetBBoxDims2D();

  /**
   * @brief Method to extract the 2D concave hull of of a defect object
   * projected into the xy plane
   * @param alpha Variable to limit the size of the resultant hull segments (the
   * smaller, the more detailed the hull). Default = 0.1
   * @return Retruns a XYZ point cloud object of the defect hull
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr GetHull2D();

  /**
   * @brief Method for returning the maximum distance across the defect area,
   * after the defect has beed projected to a 2D plane. This is important when
   * calculating OSIM severity for delaminations or spalls
   * @return Returns a float that represents the maximum distance in any
   * direction
   */
  float GetMaxDim2D();

  /**
   * @brief Pure virtual method for returning type of defect
   * @return
   */
  virtual DefectType GetType() const = 0;

  virtual DefectOSIMSeverity GetOSIMSeverity() = 0;

protected:
  // Variable for storing the defect point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr defect_cloud_ =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr defect_cloud_hull_ =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  float alpha_ = 0.1;
  bool defect_cloud_initialized_ = false, cloud_hull_calculated_ = false;
};

/** @} group defects */

} // namespace beam_defects
