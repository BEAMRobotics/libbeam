#include "beam_defects/Crack.h"
#include "beam_defects/Defect.h"
#include "beam_defects/Delam.h"
#include "beam_defects/Spall.h"

#include "beam_defects/extract_functions.h"

#include <pcl/io/pcd_io.h>

#include <iostream>
#include <string>
#include <vector>

int main() {
  // Read in the cloud data
  pcl::PCDReader reader;
  auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  reader.read("test_data/cloud_cluster_0.pcd", *cloud);

  // Example code for creating a defect
  // create delam defect with point cloud for initialization
  beam_defects::Defect::Ptr defect_ =
      std::make_shared<beam_defects::Delam>(cloud);
  std::cout << "Defect Type has size: " << defect_->GetSize() << " m^2"
            << std::endl;

  defect_->GetOSIMSeverity();

  beam_defects::Defect::Ptr defect2_ =
       std::make_shared<beam_defects::Spall>(cloud);
            std::cout << "Spall Type has size: " << defect2_->GetSize()
                      << " m^2" << std::endl;
}
