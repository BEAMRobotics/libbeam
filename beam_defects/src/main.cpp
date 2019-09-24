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
  auto cloud =
      boost::make_shared<pcl::PointCloud<beam_containers::PointBridge>>();
  reader.read("test_data/20190515_raytrace_label_PB.pcd", *cloud);

  auto cloud_cnn =
      boost::make_shared<pcl::PointCloud<beam_containers::PointBridge>>();
  reader.read("test_data/20190515_raytrace_cnn_PB.pcd", *cloud_cnn);

  // Example code for extracting defects and their size
  float threshold = 0.8;
  std::vector<beam_defects::Defect::Ptr> delam_vector_ =
      beam_defects::GetDelams(cloud, threshold);

  std::vector<beam_defects::Defect::Ptr> delam_vector_cnn =
      beam_defects::GetDelams(cloud_cnn, threshold);

  for (auto& defect : delam_vector_) {
    std::cout << "Delam size is: " << defect->GetSize() << "m^2" << std::endl;
  }

  // Code to get the matching vector
  int ind = 0;
  for (auto& defect : delam_vector_) {
    int match_ind = defect->GetMatchingDefect(delam_vector_cnn);
    std::cout << "Index: " << ind << " vs. " << match_ind << std::endl;
    std::cout << "Size: " << defect->GetSize() << " vs. "
              << delam_vector_cnn[match_ind]->GetSize() << std::endl;
    ++ind;
  }

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
