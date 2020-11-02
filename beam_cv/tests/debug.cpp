#include "beam_calibration/CameraModel.h"
#include "beam_calibration/TfTree.h"
#include "beam_cv/geometry/RelativePoseEstimator.h"
#include "beam_cv/geometry/Triangulation.h"
#include "beam_defects/Defect.h"
#include "beam_defects/Delam.h"
#include "beam_defects/extract_functions.h"
#include "beam_utils/math.hpp"
#include <beam_containers/PointBridge.h>

#include <fstream>
#include <iostream>

#include <pcl/io/pcd_io.h>

using namespace std;

void CalculateGTError(double scale);
void RelabelMask();
void QuantifyDefects();

int main(int argc, char* argv[]) {
  // double scale;
  // sscanf(argv[1], "%lf", &scale);
  // CalculateGTError(scale);
  // RelabelMask();
  // QuantifyDefects();
}

void CalculateGTError(double scale) {
  double area = 0;
  cv::Mat mask = cv::imread("/home/jake/mask.jpg", cv::IMREAD_GRAYSCALE);
  for (int i = 0; i < mask.rows; i++) {
    for (int j = 0; j < mask.cols; j++) {
      if (mask.at<uchar>(i, j) > 100) { area += scale; }
    }
  }
  std::cout << area << std::endl;
}

void RelabelMask() {
  cv::Mat mask = cv::imread("/home/jake/mask.jpg", cv::IMREAD_GRAYSCALE);
  for (int i = 0; i < mask.rows; i++) {
    for (int j = 0; j < mask.cols; j++) {
      if (mask.at<uchar>(i, j) > 140) {
        mask.at<uchar>(i, j) = 2;
      } else {
        mask.at<uchar>(i, j) = 0;
      }
    }
  }
  cv::imwrite("/home/jake/mask2.jpg", mask);
}

void QuantifyDefects() {
  pcl::PCDReader reader;
  auto cloud =
      boost::make_shared<pcl::PointCloud<beam_containers::PointBridge>>();
  reader.read("/home/jake/data/IronHorse/clouds/cam0_51_6/subsampled.pcd",
              *cloud);

  float threshold = 0.2;
  std::vector<beam_defects::Defect::Ptr> delam_vector_ =
      beam_defects::GetDelams(cloud, threshold, 0.2, 3, 80000);

  for (auto& defect : delam_vector_) {
    std::cout << "Delam size is: " << defect->GetSize() << "m^2" << std::endl;
  }
}
