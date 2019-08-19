// beam
#include "beam_cv/DepthMap.h"
#include "beam_cv/Utils.h"
// std
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>
// opencv
#include <opencv/cv.h>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace cv;
using namespace std;

// global variables
string cur_dir = "/home/jake/projects/beam_robotics/libbeam/beam_cv/"
                 "tests/test_data/";
shared_ptr<beam_calibration::CameraModel> F1;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
// functions
void TestDepthMap();

struct SuperPixel {
  vector<Point2i> pixels;
  // beam::Vec3 plane_normal;
  // beam::Vec3 plane_point;
  Point2i centroid;
  // float depth_density;
  // double avg_depth;
};

int main() {
  string intrinsics_loc = cur_dir + "F1.json";
  F1 = beam_calibration::CameraModel::LoadJSON(intrinsics_loc);
  pcl::io::loadPCDFile<pcl::PointXYZ>(cur_dir + "test.pcd", *cloud);
  TestDepthMap();
}

void TestDepthMap() {
  // load depth map
  beam_cv::DepthMap dm(F1, cloud);
  Mat viz;
  dm.ExtractDepthMap(0.001, 3);
  Mat depth_map = dm.GetDepthImage();
  viz = beam_cv::VisualizeDepthImage(depth_map);
  imwrite("/home/jake/depth.png", viz);
  // load color image
  string image_location = "/home/jake/original.jpg";
  Mat img = imread(image_location, IMREAD_COLOR);
  // preprocess image
  Mat img2 = img.clone();
  // GaussianBlur(img, img, Size(11, 11), 7, 7);
  bilateralFilter(img2, img, 11, 30, 30);

  cv::Size half(img.cols / 2, img.rows / 2);
  cv::resize(img, img, half);
  Mat src;
  cvtColor(img, src, COLOR_BGR2HSV);
  // compute superpixels
  BEAM_INFO("Computing superpixels.");
  vector<SuperPixel> superpixels;
  Ptr<ximgproc::SuperpixelSLIC> slic =
      ximgproc::createSuperpixelSLIC(src, ximgproc::SLICO, 20, 20.0);
  slic->iterate(5);
  Mat labels;
  slic->getLabels(labels);

  Mat result = img.clone();
  Mat mask;
  slic->getLabelContourMask(mask, true);
  result.setTo(Scalar(0, 0, 255), mask);
  imwrite("/home/jake/result.png", result);

  int N = slic->getNumberOfSuperpixels();
  BEAM_INFO("Superpixels computed: {}", N);
  BEAM_INFO("Filling superpixel structure...");
  superpixels.resize(N);

  for (int i = 0; i < N; ++i) {
    Mat1b sp_mask = (labels == i);
    for (int row = 0; row < sp_mask.rows; ++row) {
      for (int col = 0; col < sp_mask.cols; ++col) {
        int val = sp_mask.at<uchar>(row, col);
        if (val > 0) {
          Point2i p(row, col);
          superpixels[val].pixels.push_back(p);
        }
      }
    }
  }
  int num = 0;
  BEAM_INFO("Superpixel structure filled.");
  BEAM_INFO("Computing superpixel centroids...");
  for (auto& sp : superpixels) {
    // std::cout << sp.pixels.size() << std::endl;
    /*
    if (sp.pixels.size() > 0) {
      num++;
      int cx = 0;
      int cy = 0;
      for (Point2i p : sp.pixels) {
        cx += p.x;
        cy += p.y;
      }
      cx /= sp.pixels.size();
      cy /= sp.pixels.size();
      Point2i cent(cx, cy);
      sp.centroid = cent;
    }*/
  }
  BEAM_INFO("Centroids computed: {}", num);
  BEAM_INFO("Program finished.");
}
