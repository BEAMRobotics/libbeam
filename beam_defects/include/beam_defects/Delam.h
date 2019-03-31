#pragma once
#include "beam_defects/Defect.h"

namespace beam_defects {

/**
 * @brief Derived class for delamination defects
 */
class Delam : public Defect {
public:
  /**
   * @brief Default constructor
   */
  Delam() = default;

  /**
   * @brief Construct with a point cloud
   * @param pc
   */
  explicit Delam(pcl::PointCloud<pcl::PointXYZ>::Ptr pc);

  /**
   * @brief Default constructor
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

  // Variable for storing the defect point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr defect_cloud_ =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
};

} // namespace beam_defects
