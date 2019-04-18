#define PCL_NO_PRECOMPILE

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
  reader.read("test_data/20180622-flir_delam_ptCloud.pcd", *cloud);

  for (auto& point : cloud->points) {
    if(point.r == 255 && point.g == 0 && point.b == 0)
      {
        point.delam = 0.95;
    }
  }

  //std::vector<beam_defects::Crack> delam_vector = beam_defects::GetCracks(cloud);

  //std::cout << delam_vector[0].GetSize() << std::endl;

  // // Example code for creating a defect
  // // create delam defect with point cloud for initialization
  // beam_defects::Defect::Ptr defect_ =
  //     std::make_shared<beam_defects::Delam>(cloud);
  // std::cout << "Defect Type has size: " << defect_->GetSize() << " m^2"
  //           << std::endl;
  //
  // defect_->GetOSIMSeverity();
  //
  // beam_defects::Defect::Ptr defect2_ =
  //      std::make_shared<beam_defects::Spall>(cloud);
  //           std::cout << "Spall Type has size: " << defect2_->GetSize()
  //                     << " m^2" << std::endl;
}
