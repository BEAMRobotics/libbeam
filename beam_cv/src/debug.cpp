// beam
#include "beam_cv/DepthCompletion.h"
#include "beam_cv/DepthMap.h"
#include "beam_cv/DepthSuperpixels.h"
#include "beam_cv/LinearRegression.h"
#include "beam_cv/Utils.h"
#include "beam_utils/math.hpp"
// std
#include <chrono>
#include <cmath>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <sys/types.h>
#include <vector>
// opencv
#include <opencv/cv.h>

using namespace cv;
using namespace std;

// global variables
string cur_dir = "/home/jake/projects/beam_robotics/libbeam/beam_cv/"
                 "tests/";
string training_dir =
    "/home/jake/projects/kitti/train/2011_09_26_drive_0001_sync/proj_depth/"
    "velodyne_raw/image_02/";

string prediction_dir =
    "/home/jake/projects/kitti/prediction/2011_09_26_drive_0001_sync/image_02/";

Mat CompleteDepthImage(Mat depth_img);
Mat IPBasic(Mat depth_img);

int main() {
  /*
  char train_dir[training_dir.size() + 1];
  strcpy(train_dir, training_dir.c_str());

  struct dirent* entry;
  DIR* dir = opendir(train_dir);

  if (dir != NULL) {
    while ((entry = readdir(dir)) != NULL) {
      string depth_image_name(entry->d_name);
      string depth_image_path = training_dir + depth_image_name;
      if (depth_image_name.size() > 4) {
        std::cout << depth_image_name << std::endl;
        Mat depth_img;
        Mat depth = imread(depth_image_path, IMREAD_GRAYSCALE);
        depth.convertTo(depth_img, CV_32F);
        depth_img = CompleteDepthImage(depth_img);
        medianBlur(depth_img, depth_img, 3);
        // Mat depth_img_copy = depth_img.clone();
        // bilateralFilter(depth_img_copy, depth_img, 10, 2.5, 5);
        std::string output_location = prediction_dir + depth_image_name;
        imwrite(output_location, depth_img);
      }
    }
  }
  closedir(dir);*/

  string depth_location = cur_dir + "test_depth/depth/depth.png";
  // read in images and perform equalizaiton + smoothing
  Mat depth_img, depth_img_gt, viz;

  // read input depth and visualize it
  Mat depth = imread(depth_location, IMREAD_GRAYSCALE);
  depth.convertTo(depth_img, CV_32F);
  viz = beam_cv::VisualizeDepthImage(depth_img);
  imwrite("/home/jake/results/depth.png", viz);

  depth_img = CompleteDepthImage(depth_img);

  viz = beam_cv::VisualizeDepthImage(depth_img);
  imwrite("/home/jake/results/depth_complete.png", viz);
}

Mat CompleteDepthImage(Mat depth_img) {
  depth_img = beam_cv::DepthInterpolation(15, 3, 1, depth_img);
  depth_img = beam_cv::DepthInterpolation(10, 5, 1, depth_img);

  int window_size = 16;
  cv::Mat dst = depth_img.clone();
  dst.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance == 0) {
      int row = position[0], col = position[1];
      // calculate window start and end points
      int start_row = row - (window_size / 2);
      int start_col = col - (window_size / 2);
      int end_row = row + (window_size / 2);
      int end_col = col + (window_size / 2);
      if (start_row < 0) start_row = 0;
      if (start_col < 0) start_col = 0;
      if (end_row > dst.rows) end_row = dst.rows;
      if (end_col > dst.cols) end_col = dst.cols;
      // fill vector with neighbourhood depth values and the distance to them
      std::vector<std::tuple<float, float>> closest_points;
      for (int i = start_row; i < end_row; i++) {
        for (int j = start_col; j < end_col; j++) {
          float depth = dst.at<float>(i, j);
          if (depth != 0) {
            float dist = ((i - row) * (i - row)) + ((j - col) * (j - col));
            dist = sqrt(dist);
            closest_points.push_back(std::make_tuple(dist, depth));
          }
        }
      }

      if (closest_points.size() >= 3) {
        float idw_numerator = 0.0;
        float idw_denominator = 0.0;
        for (int i = 0; i < closest_points.size(); i++) {
          std::tuple<float, float> pair = closest_points[i];
          float depth = std::get<1>(pair);
          float dist = std::get<0>(pair);
          idw_numerator += depth / dist;
          idw_denominator += 1 / dist;
        }
        float interpolated_depth = idw_numerator / idw_denominator;
        depth_img.at<float>(row, col) = interpolated_depth;
      }
    }
  });
  return depth_img;
}

Mat IPBasic(Mat depth_img) {}