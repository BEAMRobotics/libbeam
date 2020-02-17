// beam
#include "beam_cv/DepthCompletion.h"
#include "beam_cv/DepthMap.h"
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
#include <opencv/cv.h>
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

int main() {
  string intrinsics_loc = cur_dir + "F1.json";
  F1 = beam_calibration::CameraModel::LoadJSON(intrinsics_loc);
  pcl::io::loadPCDFile<pcl::PointXYZ>(cur_dir + "test.pcd", *cloud);
  string image_location = cur_dir + "test.jpg";
  Mat img = imread(image_location, IMREAD_COLOR);

  img = beam_cv::AdaptiveHistogram(img);
  Mat img2 = img.clone();
  bilateralFilter(img2, img, 11, 30, 30);
  Mat km = beam_cv::KMeans(img, 6);
  imwrite("/home/jake/results/km.png", km);

  beam_cv::DepthMap dm(F1, cloud);
  Mat viz;
  dm.ExtractDepthMap(0.01, 1);
  Mat depth_map = dm.GetDepthImage();
  viz = beam_cv::VisualizeDepthImage(depth_map);
  imwrite("/home/jake/results/depth.png", viz);

  std::map<int, std::vector<cv::Point2i>> segments =
      beam_cv::ConnectedComponents(km);
  std::map<int, std::vector<cv::Point2i>> associated_depths;
  for (auto const& x : segments) {
    int key = x.first;
    std::vector<cv::Point2i> val = x.second;
    std::vector<cv::Point2i> depths;
    for (cv::Point2i p : val) {
      if (depth_map.at<float>(p.x, p.y) > 0.0) { depths.push_back(p); }
    }
    associated_depths.insert({key, depths});
  }

  for (auto const& x : segments) {
    int key = x.first;
    std::vector<cv::Point2i> val = x.second;
    for (cv::Point2i p : val) {
      if (depth_map.at<float>(p.x, p.y) < 0.001) {
        cv::Point2i q1, q2, q3, q4;
        float d1 = 999999, d2 = 999999, d3 = 999999, d4 = 999999;

        for (cv::Point2i dp : associated_depths[key]) {
          float dist = beam_cv::PixelDistance(dp, p);
          if (dp.x <= p.x && dp.y <= p.y && dist < d1) {
            q1 = dp;
            d1 = dist;
          } else if (dp.x <= p.x && dp.y > p.y && dist < d2) {
            q2 = dp;
            d2 = dist;
          } else if (dp.x > p.x && dp.y > p.y && dist < d3) {
            q3 = dp;
            d3 = dist;
          } else if (dp.x > p.x && dp.y <= p.y && dist < d4) {
            q4 = dp;
            d4 = dist;
          }
        }
        if (d1 < 999999 && d2 < 999999 && d3 < 999999 && d4 < 999999) {
          float idw_numerator = 0.0;
          float idw_denominator = 0.0;

          idw_numerator = depth_map.at<float>(q1.x, q1.y) / d1 +
                          depth_map.at<float>(q2.x, q2.y) / d2 +
                          depth_map.at<float>(q3.x, q3.y) / d3 +
                          depth_map.at<float>(q4.x, q4.y) / d4;

          idw_denominator = 1 / d1 + 1 / d2 + 1 / d3 + 1 / d4;

          float interpolated_depth = idw_numerator / idw_denominator;
          depth_map.at<float>(p.x, p.y) = interpolated_depth;
        }
      }
    }
  }

  viz = beam_cv::VisualizeDepthImage(depth_map);
  imwrite("/home/jake/results/depth_idw.png", viz);
}
