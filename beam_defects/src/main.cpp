#include "beam_defects/Crack.h"
#include "beam_defects/Defect.h"
#include "beam_defects/Delam.h"
#include "beam_defects/Spall.h"
#include "beam_defects/extract_functions.h"

#include <beam_containers/PointBridge.h>

#include <pcl/io/pcd_io.h>

#include <iostream>
#include <string>
#include <vector>

int main() {
  // Read in the cloud data
  pcl::PCDReader reader;
  auto cloud = boost::make_shared<pcl::PointCloud<beam_containers::PointBridge>>();
  reader.read("test_data/20190515_raytrace_label_PB.pcd", *cloud);

  // Example code for extracting defects and their size
  float threshold = 0.8;
  std::vector<beam_defects::Delam> delam_vector_ = beam_defects::GetDelams(cloud, threshold);

  for (auto& defect : delam_vector_) {
    std::cout << "Delam size is: " << defect.GetSize() << "m^2" <<std::endl;
  }

  // Check size comparison
  double comparison = delam_vector_[0].CompareSize(delam_vector_[1]);
  std::cout << comparison << std::endl;


  // Read in new cloud data
  auto cloud2 = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  reader.read("test_data/cloud_cluster_0.pcd", *cloud2);

  // Example code for creating a defect
  // create delam defect with point cloud for initialization
  beam_defects::Defect::Ptr defect_ =
      std::make_shared<beam_defects::Delam>(cloud2);
  std::cout << "Defect Type has size: " << defect_->GetSize() << " m^2"
            << std::endl;

  defect_->GetOSIMSeverity();
}
