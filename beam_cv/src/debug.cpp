// beam
#include "beam_cv/DepthCompletion.h"
#include "beam_cv/DepthMap.h"
#include "beam_cv/DepthSuperpixels.h"
#include "beam_cv/LinearRegression.h"
#include "beam_cv/Utils.h"
#include "beam_utils/math.hpp"
// std
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>
// opencv
#include <chrono>
#include <iostream>
#include <opencv/cv.h>

using namespace cv;
using namespace std;

// global variables
string cur_dir = "/home/jake/projects/beam_robotics/libbeam/beam_cv/"
                 "tests/";

int main() {
  string rgb_location = cur_dir + "test_depth/rgb/rgb.png";
  string depth_location = cur_dir + "test_depth/depth/depth.png";
  // read in images and perform equalizaiton + smoothing
  Mat depth_img, viz;
  Mat img = imread(rgb_location, IMREAD_COLOR);
  img = beam_cv::AdaptiveHistogram(img);
  Mat img2 = img.clone();
  bilateralFilter(img2, img, 11, 30, 30);
  Mat depth = imread(depth_location, IMREAD_GRAYSCALE);
  depth.convertTo(depth_img, CV_32F);
  // output given depth image
  viz = beam_cv::VisualizeDepthImage(depth_img);
  imwrite("/home/jake/results/depth.png", viz);

  auto start_time = std::chrono::high_resolution_clock::now();

  // superpixel stuff
  beam_cv::DepthSuperpixels DSP =
      beam_cv::DepthSuperpixels(img2, depth_img, true);
  std::unordered_map<int, std::shared_ptr<beam_cv::SuperPixel>> superpixels =
      DSP.GetSuperpixels();

  depth_img = beam_cv::DepthInterpolation(20, 5, 2, depth_img);
  depth_img = beam_cv::DepthInterpolation(20, 5, 2, depth_img);
  depth_img = beam_cv::DepthInterpolation(20, 5, 2, depth_img);
  depth_img = beam_cv::DepthInterpolation(20, 5, 2, depth_img);

  viz = beam_cv::VisualizeDepthImage(depth_img);
  imwrite("/home/jake/results/depth_interp.png", viz);

  for (auto& it : superpixels) {
    /*
    std::shared_ptr<beam_cv::SuperPixel> sp = it.second;
    float mean = 0;
    for (std::tuple<cv::Point2i, float> depth_point : sp->depth_values_2) {
      mean += std::get<1>(depth_point);
    }
    mean = mean / sp->depth_values_2.size();
    for (cv::Point2i p : sp->pixels) { depth_img.at<float>(p.x, p.y) = mean; }*/

    std::shared_ptr<beam_cv::SuperPixel> sp = it.second;
    for (cv::Point2i p : sp->pixels) {
      cv::Point2i p1, p2, p3, p4;
      float dp1 = 1000, dp2 = 1000, dp3 = 1000, dp4 = 1000;
      if (depth_img.at<float>(p.x, p.y) == 0) {
        for (std::tuple<cv::Point2i, float> depth_point : sp->depth_values_2) {
          cv::Point2i search_point = std::get<0>(depth_point);
          float dist = ((p.x - search_point.x) * (p.x - search_point.x)) +
                       ((p.y - search_point.y) * (p.y - search_point.y));
          dist = sqrt(dist);
          if (search_point.x < p.x && search_point.y < p.y && dist < dp1) {
            dp1 = dist;
            p1 = search_point;
          } else if (search_point.x < p.x && search_point.y >= p.y &&
                     dist < dp2) {
            dp2 = dist;
            p2 = search_point;
          } else if (search_point.x >= p.x && search_point.y >= p.y &&
                     dist < dp3) {
            dp3 = dist;
            p3 = search_point;
          } else if (search_point.x >= p.x && search_point.y < p.y &&
                     dist < dp4) {
            dp4 = dist;
            p4 = search_point;
          }
        }
        // find closest in each quadrant
        // inverse distance weighted averaging to interpolate
        // if only 2 found, copy the value of one of them
      }
      if (dp1 != 1000 && dp2 != 1000 && dp3 != 1000 && dp4 != 1000) {
        float p1_f = depth_img.at<float>(p1.x, p1.y);
        float p2_f = depth_img.at<float>(p2.x, p2.y);
        float p3_f = depth_img.at<float>(p3.x, p3.y);
        float p4_f = depth_img.at<float>(p4.x, p4.y);
        depth_img.at<float>(p.x, p.y) = (p1_f + p2_f + p3_f + p4_f) / 4;
      } else if (dp1 < dp2 && dp1 < dp3 && dp1 < dp4) {
        depth_img.at<float>(p.x, p.y) = depth_img.at<float>(p1.x, p1.y);
      } else if (dp2 < dp1 && dp2 < dp3 && dp1 < dp4) {
        depth_img.at<float>(p.x, p.y) = depth_img.at<float>(p2.x, p2.y);
      } else if (dp3 < dp2 && dp3 < dp1 && dp1 < dp4) {
        depth_img.at<float>(p.x, p.y) = depth_img.at<float>(p3.x, p3.y);
      } else if (dp4 < dp2 && dp4 < dp1 && dp4 < dp3) {
        depth_img.at<float>(p.x, p.y) = depth_img.at<float>(p4.x, p4.y);
      }
    }
  }
  cv::morphologyEx(depth_img, depth_img, cv::MORPH_CLOSE,
                   beam::GetFullKernel(3));
  auto end_time = std::chrono::high_resolution_clock::now();
  auto time = end_time - start_time;
  std::cout << time / std::chrono::milliseconds(1) << "ms to run. (total)\n";

  viz = beam_cv::VisualizeDepthImage(depth_img);
  imwrite("/home/jake/results/depth_interp2.png", viz);
}

/*
Mat depth_completed = depth_img.clone();
for (auto& it : superpixels) {
  std::cout << it.first << "/" << DSP.num_superpixels << std::endl;
  std::shared_ptr<beam_cv::SuperPixel> sp = it.second;
  Eigen::MatrixXf X = std::get<0>(sp->depth_values);
  Eigen::RowVectorXf response = std::get<1>(sp->depth_values);
  if (X.rows() >= 6) {
    Eigen::MatrixXf X_prime(X.rows(), 9);
    for (int i = 0; i < X.rows(); i++) {
      double x = X(i, 0), y = X(i, 1), depth = response(i);
      double x3 = x * x * x, x2y = x * x * y, xy2 = x * y * y, y3 = y * y * y;
      double x2 = x * x, xy = x * y, y2 = y * y;
      X_prime(i, 0) = x;
      X_prime(i, 1) = y;
      X_prime(i, 2) = x3;
      X_prime(i, 3) = x2y;
      X_prime(i, 4) = xy2;
      X_prime(i, 5) = y3;
      X_prime(i, 6) = x2;
      X_prime(i, 7) = xy;
      X_prime(i, 8) = y2;
    }

    // Regression Variables
    int max_iteration = 10000;
    double learning_rate = 0.00001;

    // Training
    // std::cout << "Training \n";
    beam_cv::Dataset data =
        beam_cv::Dataset(X_prime, response, X_prime.rows(), 9);
    beam_cv::LinearRegression linear_reg = beam_cv::LinearRegression(data);
    linear_reg.Train(max_iteration, learning_rate);
    // linear_reg.PrintWeights();

    for (Point2i pix : sp->pixels) {
      double x = pix.x, y = pix.y;
      double x3 = x * x * x, x2y = x * x * y, xy2 = x * y * y, y3 = y * y * y;
      double x2 = x * x, xy = x * y, y2 = y * y;
      Eigen::VectorXf x_test(9);
      x_test << x, y, x3, x2y, xy2, y3, x2, xy, y2;
      double y_test = linear_reg.Predict(x_test);
      depth_completed.at<float>(pix.x, pix.y) = y_test;
    }
  }
}

viz = beam_cv::VisualizeDepthImage(depth_completed);
imwrite("/home/jake/depth_sp.png", viz);*/

/**
 *
 *
 *
  // perform grid sampling for low res, high density map
  int width = depth_interp.cols;
  int height = depth_interp.rows;
  int grid_pixel = 5;
  Mat grid = Mat::zeros(cv::Size(width, height), CV_8UC1);
  Mat low_res = depth_interp.clone();
  low_res.forEach<float>(
      [&](float& distance, const int* position) -> void { distance = 0; });
  for (int i = 0; i < height - 2; i++) {
    for (int j = 0; j < width - 2; j++) {
      if ((i + 2) % 5 == 0 && (j + 2) % 5 == 0) {
        grid.at<uchar>(i, j) = 255;
        std::vector<double> depth_vals;
        for (int win_row = i - 2; win_row <= i + 2; win_row++) {
          for (int win_col = j - 2; win_col <= j + 2; win_col++) {
            double val = depth_interp.at<float>(win_row, win_col);
            if (val != 0) { depth_vals.push_back(val); }
          }
        }
        if (depth_vals.size() > 0) {
          double median = depth_vals[int(depth_vals.size() / 2) + 1];
          // std::cout << i << ";" << j << std::endl;
          for (int win_row = i - 2; win_row <= i + 2; win_row++) {
            for (int win_col = j - 2; win_col <= j + 2; win_col++) {
              low_res.at<float>(win_row, win_col) = median;
            }
          }
        }
      }
    }
  }
  viz = beam_cv::VisualizeDepthImage(low_res);
  imwrite("/home/jake/grid.png", viz);*/

/*
 // perform depth interpolation
 auto start_time = std::chrono::high_resolution_clock::now();
 depth_img = beam_cv::DepthInterpolation(20, 5, 2, depth_img);
 // depth_interp = beam_cv::DepthInterpolation(10, 2, 1.5, depth_interp);
 // cv::morphologyEx(depth_interp, depth_interp, cv::MORPH_CLOSE,
 //               beam::GetEllipseKernel(3));
 // Mat depth_filtered = depth_interp.clone();
 // cv::bilateralFilter(depth_interp, depth_filtered, 5, 3, 5);
 // cv::morphologyEx(depth_interp, depth_interp, cv::MORPH_CLOSE,
 //                beam::GetFullKernel(5));
 auto end_time = std::chrono::high_resolution_clock::now();
 auto time = end_time - start_time;
 std::cout << time / std::chrono::milliseconds(1) << "ms to run.\n";
 // output interpolated depth image
 viz = beam_cv::VisualizeDepthImage(depth_img);
 imwrite("/home/jake/depth_interp.png", viz);*/

/*
for (auto& it : superpixels) {
  std::shared_ptr<beam_cv::SuperPixel> sp = it.second;
  Eigen::RowVectorXf depths = std::get<1>(sp->depth_values);
  int size = depths.cols();
  if (size > 0) {
    // calculate median
    float median;
    if (size % 2 == 0) {
      median = depths(size / 2);
    } else {
      median = (depths(int(size / 2) - 1) + depths(int(size / 2) + 1)) / 2;
    }
    // calculate standard deviation
    float var = 0;
    for (uint32_t n = 0; n < size; n++) {
      var = var + (depths(n) - median) * (depths(n) - median);
    }
    var /= size;
    float sd = sqrt(var);
    // remove points from depth image
    std::vector<std::tuple<cv::Point2i, float>> depth_points =
        sp->depth_values_2;
    for (int i = 0; i < sp->depth_values_2.size(); i++) {
      std::tuple<cv::Point2i, float> tup = sp->depth_values_2[i];
      cv::Point2i pixel = std::get<0>(tup);
      float d = std::get<1>(tup);
      if (d < median - 1.5 * sd || d > median + 1.5 * sd) {
        depth_img.at<float>(pixel.x, pixel.y) = 0;
      }
    }
  }
}

DSP = beam_cv::DepthSuperpixels(img2, depth_img, true);
viz = beam_cv::VisualizeDepthImage(depth_img);
imwrite("/home/jake/depth_filtered.png", viz);
*/