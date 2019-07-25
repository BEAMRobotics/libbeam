#include "beam_cv/DepthMap.h"
#include "beam_cv/Morphology.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/time.h>
#include <string>

using namespace cv;
using namespace std;

void TestDepthMap();
int TestCrackCalculation(int argc, char** argv);
void fillPosesVector(std::string file);
std::vector<Eigen::Affine3d> poses;
std::vector<ros::Time> poses_time;

int main(int argc, char** argv) {
  TestCrackCalculation(argc, argv);
  // TestDepthMap();
}

void TestDepthMap() {
  std::string cur_dir = "/home/jake/projects/beam_robotics/libbeam/beam_cv";
  // load intrinsics
  std::string intrinsics_location = cur_dir + "/tests/test_data/F1.json";
  std::shared_ptr<beam_calibration::CameraModel> F1 =
      beam_calibration::CameraModel::LoadJSON(intrinsics_location);
  // load pcd
  std::string pcd_location = cur_dir + "/tests/test_data/test.pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_location, *cloud);

  cv::Mat kernel = beam::GetCrossKernel(5);
  cv::Mat kernel2 = beam::GetEllipseKernel(5);
  /*
   * Test depth map filling
   */
  beam_cv::DepthMap dm(F1, cloud);
  cv::Mat di_viz;

  dm.ExtractDepthMap(0.001, 3);
  di_viz = dm.VisualizeDepthImage();
  cv::imwrite("/home/jake/ext.png", di_viz);
  dm.DepthInterpolation(70, 5, 0.05, 4);
  di_viz = dm.VisualizeDepthImage();
  cv::imwrite("/home/jake/interp.png", di_viz);
  dm.DepthCompletion(kernel);
  di_viz = dm.VisualizeDepthImage();
  cv::imwrite("/home/jake/comp.png", di_viz);
  /*
  beam_cv::DepthMap dm2(F1, new_cloud);
  dm2.ExtractDepthMap(0.02, 21);
  di_viz = dm2.VisualizeDepthImage();
  cv::imwrite("/home/jake/ext2.png", di_viz);

  dm2.DepthCompletion(kernel);
  di_viz = dm2.VisualizeDepthImage();
  cv::imwrite("/home/jake/completed.png", di_viz);
  dm2.VerticalDepthExtrapolation();
  di_viz = dm2.VisualizeDepthImage();
  cv::imwrite("/home/jake/extrapolated.png", di_viz);

  beam_cv::DepthMap dm3(F1, cloud);
  dm3.ExtractDepthMap(0.02, 31);
  di_viz = dm3.VisualizeDepthImage();
  cv::imwrite("/home/jake/extracted.png", di_viz);
  dm3.DepthCompletion(kernel);
  di_viz = dm3.VisualizeDepthImage();
  cv::imwrite("/home/jake/full.png", di_viz);*/
}

int TestCrackCalculation(int argc, char** argv) {
  Mat image = imread("/home/jake/Downloads/crack-mask1.png", 0);
  std::string cur_dir = "/home/jake/projects/beam_robotics/libbeam/beam_cv";
  // load intrinsics
  std::string intrinsics_location = cur_dir + "/tests/test_data/F1.json";
  std::shared_ptr<beam_calibration::CameraModel> F1 =
      beam_calibration::CameraModel::LoadJSON(intrinsics_location);
  // load pcd
  std::string pcd_location = cur_dir + "/tests/test_data/test.pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_location, *cloud);
  cv::Mat kernel2 = beam::GetEllipseKernel(5);
  beam_cv::DepthMap dm(F1, cloud);
  dm.ExtractDepthMap(0.02, 31);
  dm.DepthCompletion(kernel2);
  // process image used (just temp with example image)
  cv::threshold(image, image, 0, 255, cv::THRESH_BINARY);
  image = beam_cv::RemoveClusters(image, 100);
  cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT, Size(3, 3));
  morphologyEx(image, image, cv::MORPH_CLOSE, element2);
  std::vector<cv::Mat> cracks = beam_cv::SegmentComponents(image);
  cv::Mat chosen_crack = cracks[2];
  cv::Mat skeleton = beam_cv::ExtractSkeleton(chosen_crack);
  /*

  make this a function to find start_point
  // returns start point
  */
  int window_width = 11;
  beam::Vec2 startp;
  beam::Vec2 endp;
  bool found = false;
  // use normal iteration so you always get the furthest left/top
  skeleton.forEach<uchar>([&](uchar& pixel, const int* position) -> void {
    if (pixel == 255) {
      int sur_zeros = 0;
      uchar up, down, left, right, upleft, upright, downleft, downright;
      up = skeleton.at<uchar>(position[0] - 1, position[1]);
      upleft = skeleton.at<uchar>(position[0] - 1, position[1] - 1);
      upright = skeleton.at<uchar>(position[0] - 1, position[1] + 1);
      down = skeleton.at<uchar>(position[0] + 1, position[1]);
      downleft = skeleton.at<uchar>(position[0] + 1, position[1] - 1);
      downright = skeleton.at<uchar>(position[0] + 1, position[1] + 1);
      left = skeleton.at<uchar>(position[0], position[1] - 1);
      right = skeleton.at<uchar>(position[0], position[1] + 1);
      if (up == 0) { sur_zeros++; }
      if (down == 0) { sur_zeros++; }
      if (left == 0) { sur_zeros++; }
      if (right == 0) { sur_zeros++; }
      if (downleft == 0) { sur_zeros++; }
      if (downright == 0) { sur_zeros++; }
      if (upleft == 0) { sur_zeros++; }
      if (upright == 0) { sur_zeros++; }
      if (sur_zeros > 6 && !found) {
        startp[0] = position[0];
        startp[1] = position[1];
        found = true;
      }
    }
  });

  /*

  this is also a function to find endpoint
  //takes in startpoint to find endpoint
  */
  int start_row = startp[0], end_row = startp[0] + window_width;
  int start_col = startp[1] - (window_width - 1) / 2,
      end_col = startp[1] + (window_width - 1) / 2;
  for (int row = start_row; row < end_row; row++) {
    if (skeleton.at<uchar>(row, start_col) == 255) {
      endp[0] = row;
      endp[1] = start_col;
    } else {
      skeleton.at<uchar>(row, start_col) = 100;
    }
  }
  for (int row = start_row; row < end_row; row++) {
    if (skeleton.at<uchar>(row, end_col) == 255) {
      endp[0] = row;
      endp[1] = end_col;
    } else {
      skeleton.at<uchar>(row, end_col) = 100;
    }
  }
  for (int col = start_col; col < end_col; col++) {
    if (skeleton.at<uchar>(end_row, col) == 255) {
      endp[0] = end_row;
      endp[1] = col;
    } else {
      skeleton.at<uchar>(end_row, col) = 100;
    }
  }

  /*

  this should be a function to return width of the current window
  // takes in both start and endpoint
  */
  float slope = (endp[0] - startp[0]) / (startp[1] - endp[1]);
  if (abs(slope) > 0) {
    int row = (start_row + end_row) / 2;
    beam::Vec2 p1, p2;
    for (int col = start_col; col < end_col; col++) {
      if (chosen_crack.at<uchar>(row, col) == 255) {
        beam::Vec2 pixel(row, col);
        p1 = pixel;
      }
    }
    for (int col = end_col; col > start_col; col--) {
      if (chosen_crack.at<uchar>(row, col) == 255) {
        beam::Vec2 pixel(row, col);
        p2 = pixel;
      }
    }
    std::cout << dm.GetDistance(p1, p2) << std::endl;
    // measure horizontal
  } else if (abs(slope) < 0) {
    // measure vertical
  } else {
    // measure on slant
  }
  cv::imshow("skeleton", skeleton);
  cv::waitKey(0);
}