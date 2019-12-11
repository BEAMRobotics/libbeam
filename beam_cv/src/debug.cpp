// beam
#include "beam_calibration/TfTree.h"
#include "beam_cv/DepthMap.h"
#include "beam_cv/DepthSuperpixels.h"
#include "beam_cv/LinearRegression.h"
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

using namespace cv;
using namespace std;

// global variables
string cur_dir = "/home/jake/projects/beam_robotics/libbeam/beam_cv/"
                 "tests/";

int main() {
  string rgb_location = cur_dir + "test_data/rgb.png";
  string depth_location = cur_dir + "test_data/depth.png";

  Mat img = imread(rgb_location, IMREAD_COLOR);
  Mat depth = imread(depth_location, IMREAD_GRAYSCALE);
  Mat depth_img;
  depth.convertTo(depth_img, CV_32F);

  // preprocess image
  img = beam_cv::AdaptiveHistogram(img);
  Mat img2 = img.clone();
  bilateralFilter(img2, img, 11, 30, 30);
  Mat src;
  cvtColor(img, src, COLOR_BGR2HSV);

  beam_cv::DepthSuperpixels DSP = beam_cv::DepthSuperpixels(img, depth_img);
  std::unordered_map<int, std::shared_ptr<beam_cv::SuperPixel>> superpixels =
      DSP.GetSuperpixels();

  // load depth map
  beam_cv::DepthMap dm;
  dm.SetDepthImage(depth_img);
  Mat depth_map = dm.GetDepthImage();
  Mat viz = beam_cv::VisualizeDepthImage(depth_map);
  imwrite("/home/jake/depth.png", viz);
}
