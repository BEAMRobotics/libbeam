#include "beam_cv/DepthMap.h"
#include "beam_cv/Utils.h"
#include "beam_utils/uf.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <ros/time.h>
#include <string>

using namespace cv;
using namespace std;

void TestDepthMap();
cv::Mat kernel = beam::GetFullKernel(5);

int main() {
  TestDepthMap();
}

float distance(cv::Point2i p1, cv::Point2i p2) {
  float dist =
      sqrt(((p1.x - p2.x) * (p1.x - p2.x)) + ((p1.y - p2.y) * (p1.y - p2.y)));
  return dist;
}

void TestDepthMap() {
  std::string cur_dir =
      "/home/jake/projects/beam_robotics/libbeam/beam_cv/tests/test_data/";
  // load intrinsics
  std::string intrinsics_location = cur_dir + "F1.json";
  std::shared_ptr<beam_calibration::CameraModel> F1 =
      beam_calibration::CameraModel::LoadJSON(intrinsics_location);
  // load pcd
  std::string pcd_location = cur_dir + "test.pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_location, *cloud);

  // load depth map
  beam_cv::DepthMap dm(F1, cloud);
  cv::Mat di_viz;
  dm.ExtractDepthMap(0.001, 3);
  // dm.DepthInterpolation(70, 5, 0.1, 2);
  // dm.DepthMeshing();
  std::shared_ptr<cv::Mat> depth_image = dm.GetDepthImage();
  di_viz = beam_cv::VisualizeDepthImage(*depth_image);
  cv::imwrite("/home/jake/mesh.png", di_viz);

  cv::Mat completed = cv::Mat::zeros(F1->GetHeight(), F1->GetWidth(), CV_32FC1);
  // load color image
  std::string image_location = "/home/jake/original.jpg";
  cv::Mat img = cv::imread(image_location, IMREAD_COLOR);
  cv::Mat comp1 = dm.KMeansCompletion(11, img);
  cv::Mat comp2 = dm.KMeansCompletion(7, img);
  cv::Mat comp3 = dm.KMeansCompletion(5, img);

  comp1.forEach<float>([&](float& distance, const int* position) -> void {
    float comp2_dist = comp2.at<float>(position[0], position[1]);
    float comp3_dist = comp3.at<float>(position[0], position[1]);
    if (distance == 0.0 && comp2_dist != 0.0) {
      distance = comp2_dist;
    } else if (distance == 0.0 && comp3_dist != 0.0) {
      distance = comp3_dist;
    }
  });
  di_viz = beam_cv::VisualizeDepthImage(comp1);
  cv::imwrite("/home/jake/comp1.png", di_viz);

  BEAM_INFO("Program finished.");
}
