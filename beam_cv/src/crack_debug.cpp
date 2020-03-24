// beam
#include "beam_calibration/TfTree.h"
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
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace cv;
using namespace std;
#define BOTTOM 1
#define TOP 2
#define LEFT 3
#define RIGHT 4
// functions
std::vector<cv::Mat> SegmentMultiscale(cv::Mat depth_image);
beam::Mat4 InterpolateTransformationMatrices(beam::Mat4 C1, beam::Mat4 C2,
                                             float alpha);
void TransformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
// global variables
string cur_dir = "/home/jake/projects/beam_robotics/libbeam/beam_cv/"
                 "tests/test_data/";

// crack eval functions
beam::Vec2 GetStartPoint();
beam::Vec2 GetEndPoint(beam::Vec2 startp, int window);
beam::Vec2 GetAbsEndPoint();
void ComputeWindow(beam::Vec2 startp, int side);
double GetWindowWidth(beam::Vec2 startp, beam::Vec2 endp);
int GetWindowSide(beam::Vec2 endp, std::vector<int> window);
// variables
int window_width_ = 41;
cv::Mat crack_, skeleton_;
std::vector<int> window_{0, 0, 0, 0};
beam_cv::DepthMap dm;
cv::Mat kernel = beam::GetEllipseKernel(5);

/*
 *
 *
 *
 */
int main() {
  // load point cloud and camera intrinsics
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  string intrinsics_loc = cur_dir + "F2.json";
  shared_ptr<beam_calibration::CameraModel> model =
      beam_calibration::CameraModel::LoadJSON(intrinsics_loc);
  pcl::io::loadPCDFile<pcl::PointXYZ>("/home/jake/Thesis/pearson.pcd", *cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud1(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud2(
      new pcl::PointCloud<pcl::PointXYZ>());
  // perform point cloud transformation to pose
  beam_calibration::TfTree Tree;
  std::string extrinsics_loc = "/home/jake/Thesis/extrinsics_positive_90.json";
  Tree.LoadJSON(extrinsics_loc);
  std::string to_frame = "F2_link";
  std::string from_frame = "hvlp_link";
  beam::Mat4 T_F2_hvlp = Tree.GetTransformEigen(to_frame, from_frame).matrix();
  beam::Mat4 C1;
  C1 << 0.08022717168086207, -0.9951101526619915, 0.05761410411624448,
      1.508715640926088, 0.996269037651813, 0.07820792998429027,
      -0.03649005760346314, 24.18400067119094, 0.03180574697159621,
      0.06032664217905979, 0.9976718351757645, 0.4598058829275563, 0.0, 0.0,
      0.0, 1.0;
  beam::Mat4 C2;
  C2 << 0.08284143797290246, -0.9950077071098744, 0.05565032746114354,
      1.530574006305773, 0.9960852707262051, 0.08094413367800235,
      -0.03552718206416662, 24.51237906495184, 0.03084525242050206,
      0.05837559434445578, 0.9978180497405565, 0.4880504840839431, 0.0, 0.0,
      0.0, 1.0;
  beam::Mat4 C = InterpolateTransformationMatrices(C1, C2, 0.5);
  beam::Mat4 rotation;
  rotation << 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;
  beam::Mat4 T_F2_map = T_F2_hvlp * C.inverse();
  pcl::transformPointCloud(*cloud, *transformed_cloud1, T_F2_map);
  pcl::transformPointCloud(*transformed_cloud1, *transformed_cloud2, rotation);
  // rgb image stuff
  string image_location = "/home/jake/BGRImage.jpg";
  Mat img = imread(image_location, IMREAD_COLOR);
  img = beam_cv::AdaptiveHistogram(img);
  img = model->UndistortImage(img);
  Mat img2 = img.clone();
  bilateralFilter(img2, img, 11, 30, 30);
  imwrite("/home/jake/results/image.png", img);
  // intialize, extract and complete depth image
  beam_cv::DepthMap dm(model, transformed_cloud2);
  Mat viz;
  dm.ExtractDepthMap(0.01, 1);
  Mat depth_map = dm.GetDepthImage();
  viz = beam_cv::VisualizeDepthImage(depth_map);
  imwrite("/home/jake/results/depth.png", viz);
  depth_map = beam_cv::MultiscaleInterpolation(depth_map);
  dm.SetDepthImage(depth_map);
  viz = beam_cv::VisualizeDepthImage(depth_map);
  imwrite("/home/jake/results/depth_complete.png", viz);

  // read crack, extract skelon
  crack_ = imread("/home/jake/results/crack.png", 0);
  // process image used (just temp with example image)
  cv::threshold(crack_, crack_, 0, 255, cv::THRESH_BINARY);
  skeleton_ = beam_cv::ExtractSkeleton(crack_);
  std::vector<beam::Vec2> skel_points;
  std::vector<double> widths;
  // get initial start point
  beam::Vec2 absendp = GetAbsEndPoint();
  beam::Vec2 startp = GetStartPoint();
  beam::Vec2 endp;
  // get endpoint from each type of window
  std::vector<beam::Vec2> endpts{
      GetEndPoint(startp, TOP), GetEndPoint(startp, BOTTOM),
      GetEndPoint(startp, LEFT), GetEndPoint(startp, RIGHT)};
  // find valid endpoint and corresponding window
  for (int i = 0; i < endpts.size(); i++) {
    beam::Vec2 point = endpts[i];
    if (point[0] != 0 && point[1] != 0) {
      if (i == 0) {
        endp = point;
        ComputeWindow(startp, TOP);
      } else if (i == 1) {
        endp = point;
        ComputeWindow(startp, BOTTOM);
      } else if (i == 2) {
        endp = point;
        ComputeWindow(startp, LEFT);
      } else if (i == 3) {
        endp = point;
        ComputeWindow(startp, RIGHT);
      }
    }
  }

  while (endp[0] != 0 && endp[1] != 0) {
    skel_points.push_back(startp);
    // double width = GetWindowWidth(startp, endp);
    // std::cout << width << std::endl;
    int side = GetWindowSide(endp, window_);
    startp = endp;
    int new_window;
    if (side == RIGHT) {
      new_window = LEFT;
    } else if (side == BOTTOM) {
      new_window = TOP;
    } else if (side == TOP) {
      new_window = BOTTOM;
    } else if (side == LEFT) {
      new_window = RIGHT;
    }
    endp = GetEndPoint(startp, new_window);
  }
  skel_points.push_back(absendp);

  double total_dist = 0.0;
  for (int i = 0; i < skel_points.size() - 1; i++) {
    double dist = dm.GetDistance(skel_points[i], skel_points[i + 1]);
    total_dist += dist;
  }
  std::cout << total_dist << std::endl;
  // find absolute endpoint of crack and add after while loop breaks
  cv::imwrite("/home/jake/skeleton.png", skeleton_);
  cv::waitKey(0);
}

beam::Mat4 InterpolateTransformationMatrices(beam::Mat4 C1, beam::Mat4 C2,
                                             float alpha) {
  beam::Mat3 C1_r;
  C1_r << C1(0, 0), C1(0, 1), C1(0, 2), C1(1, 0), C1(1, 1), C1(1, 2), C1(2, 0),
      C1(2, 1), C1(2, 2);

  beam::Mat3 C2_r;
  C2_r << C2(0, 0), C2(0, 1), C2(0, 2), C2(1, 0), C2(1, 1), C2(1, 2), C2(2, 0),
      C2(2, 1), C2(2, 2);
  beam::Vec3 C1_t;
  C1_t << C1(0, 3), C1(1, 3), C1(2, 3);
  beam::Vec3 C2_t;
  C2_t << C2(0, 3), C2(1, 3), C2(2, 3);

  beam::Mat3 C_r = (C2_r * C1_r.transpose()).pow(alpha) * C1_r;
  beam::Vec3 C_t;
  C_t << (C1_t(0, 0) + C2_t(0, 0)) / 2, (C1_t(1, 0) + C2_t(1, 0)) / 2,
      (C1_t(2, 0) + C2_t(2, 0)) / 2;

  beam::Mat4 C;
  C << C_r(0, 0), C_r(0, 1), C_r(0, 2), C_t(0, 0), C_r(1, 0), C_r(1, 1),
      C_r(1, 2), C_t(1, 0), C_r(2, 0), C_r(2, 1), C_r(2, 2), C_t(2, 0), 0, 0, 0,
      1;
  return C;
}

void TransformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud1(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud2(
      new pcl::PointCloud<pcl::PointXYZ>());

  beam_calibration::TfTree Tree;
  std::string extrinsics_loc = "/home/jake/Thesis/extrinsics_positive_90.json";
  Tree.LoadJSON(extrinsics_loc);
  std::string to_frame = "F2_link";
  std::string from_frame = "hvlp_link";
  beam::Mat4 T_F2_hvlp = Tree.GetTransformEigen(to_frame, from_frame).matrix();

  beam::Mat4 C1;
  C1 << 0.08022717168086207, -0.9951101526619915, 0.05761410411624448,
      1.508715640926088, 0.996269037651813, 0.07820792998429027,
      -0.03649005760346314, 24.18400067119094, 0.03180574697159621,
      0.06032664217905979, 0.9976718351757645, 0.4598058829275563, 0.0, 0.0,
      0.0, 1.0;
  beam::Mat4 C2;
  C2 << 0.08284143797290246, -0.9950077071098744, 0.05565032746114354,
      1.530574006305773, 0.9960852707262051, 0.08094413367800235,
      -0.03552718206416662, 24.51237906495184, 0.03084525242050206,
      0.05837559434445578, 0.9978180497405565, 0.4880504840839431, 0.0, 0.0,
      0.0, 1.0;

  beam::Mat4 C = InterpolateTransformationMatrices(C1, C2, 0.5);
  beam::Mat4 rotation;
  rotation << 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;
  beam::Mat4 T_F2_map = T_F2_hvlp * C.inverse();
  pcl::transformPointCloud(*cloud, *transformed_cloud1, T_F2_map);
  pcl::transformPointCloud(*transformed_cloud1, *transformed_cloud2, rotation);
}

void ComputeWindow(beam::Vec2 startp, int side) {
  if (side == TOP) {
    window_[0] = (startp[0]);
    window_[1] = (startp[0] + window_width_);
    window_[2] = (startp[1] - (window_width_ - 1) / 2);
    window_[3] = (startp[1] + (window_width_ - 1) / 2);
  } else if (side == BOTTOM) {
    window_[0] = (startp[0] - window_width_);
    window_[1] = (startp[0]);
    window_[2] = (startp[1] - (window_width_ - 1) / 2);
    window_[3] = (startp[1] + (window_width_ - 1) / 2);
  } else if (side == RIGHT) {
    window_[0] = (startp[0] - (window_width_ - 1) / 2);
    window_[1] = (startp[0] + (window_width_ - 1) / 2);
    window_[2] = (startp[1] - window_width_);
    window_[3] = (startp[1]);
  } else if (side == LEFT) {
    window_[0] = (startp[0] - (window_width_ - 1) / 2);
    window_[1] = (startp[0] + (window_width_ - 1) / 2);
    window_[2] = (startp[1]);
    window_[3] = (startp[1] + window_width_);
  }
}

beam::Vec2 GetStartPoint() {
  beam::Vec2 startp;
  bool found = false;
  // use normal iteration so you always get the furthest left/top
  for (int row = 0; row < skeleton_.rows; row++) {
    for (int col = 0; col < skeleton_.cols; col++) {
      uchar pixel = skeleton_.at<uchar>(row, col);
      if (pixel == 255 && !found) {
        int sur_zeros = 0;
        uchar up, down, left, right, upleft, upright, downleft, downright;
        up = skeleton_.at<uchar>(row - 1, col);
        upleft = skeleton_.at<uchar>(row - 1, col - 1);
        upright = skeleton_.at<uchar>(row - 1, col + 1);
        down = skeleton_.at<uchar>(row + 1, col);
        downleft = skeleton_.at<uchar>(row + 1, col - 1);
        downright = skeleton_.at<uchar>(row + 1, col + 1);
        left = skeleton_.at<uchar>(row, col - 1);
        right = skeleton_.at<uchar>(row, col + 1);
        if (up == 0) { sur_zeros++; }
        if (down == 0) { sur_zeros++; }
        if (left == 0) { sur_zeros++; }
        if (right == 0) { sur_zeros++; }
        if (downleft == 0) { sur_zeros++; }
        if (downright == 0) { sur_zeros++; }
        if (upleft == 0) { sur_zeros++; }
        if (upright == 0) { sur_zeros++; }
        if (sur_zeros > 6) {
          startp[0] = row;
          startp[1] = col;
          found = true;
        }
      }
    }
  }
  return startp;
}

beam::Vec2 GetAbsEndPoint() {
  beam::Vec2 endp;
  bool found = false;
  // use normal iteration so you always get the furthest left/top
  for (int row = skeleton_.rows; row > 0; row--) {
    for (int col = skeleton_.cols; col > 0; col--) {
      uchar pixel = skeleton_.at<uchar>(row, col);
      if (pixel == 255 && !found) {
        int sur_zeros = 0;
        uchar up, down, left, right, upleft, upright, downleft, downright;
        up = skeleton_.at<uchar>(row - 1, col);
        upleft = skeleton_.at<uchar>(row - 1, col - 1);
        upright = skeleton_.at<uchar>(row - 1, col + 1);
        down = skeleton_.at<uchar>(row + 1, col);
        downleft = skeleton_.at<uchar>(row + 1, col - 1);
        downright = skeleton_.at<uchar>(row + 1, col + 1);
        left = skeleton_.at<uchar>(row, col - 1);
        right = skeleton_.at<uchar>(row, col + 1);
        if (up == 0) { sur_zeros++; }
        if (down == 0) { sur_zeros++; }
        if (left == 0) { sur_zeros++; }
        if (right == 0) { sur_zeros++; }
        if (downleft == 0) { sur_zeros++; }
        if (downright == 0) { sur_zeros++; }
        if (upleft == 0) { sur_zeros++; }
        if (upright == 0) { sur_zeros++; }
        if (sur_zeros > 6) {
          endp[0] = row;
          endp[1] = col;
          found = true;
        }
      }
    }
  }
  return endp;
}

beam::Vec2 GetEndPoint(beam::Vec2 startp, int window) {
  beam::Vec2 endp(0, 0);
  ComputeWindow(startp, window);
  int start_row = window_[0], end_row = window_[1];
  int start_col = window_[2], end_col = window_[3];
  // left col
  if (window != LEFT) {
    for (int row = start_row; row <= end_row; row++) {
      if (skeleton_.at<uchar>(row, start_col) == 255) {
        endp[0] = row;
        endp[1] = start_col;
      } else {
        skeleton_.at<uchar>(row, start_col) = 100;
      }
    }
  }
  // right col
  if (window != RIGHT) {
    for (int row = start_row; row <= end_row; row++) {
      if (skeleton_.at<uchar>(row, end_col) == 255) {
        endp[0] = row;
        endp[1] = end_col;
      } else {
        skeleton_.at<uchar>(row, end_col) = 100;
      }
    }
  }
  // bottom row
  if (window != BOTTOM) {
    for (int col = start_col; col <= end_col; col++) {
      if (skeleton_.at<uchar>(end_row, col) == 255) {
        endp[0] = end_row;
        endp[1] = col;
      } else {
        skeleton_.at<uchar>(end_row, col) = 100;
      }
    }
  }
  // top row
  if (window != TOP) {
    for (int col = start_col; col <= end_col; col++) {
      if (skeleton_.at<uchar>(start_row, col) == 255) {
        endp[0] = start_row;
        endp[1] = col;
      } else {
        skeleton_.at<uchar>(start_row, col) = 100;
      }
    }
  }
  return endp;
}

double GetWindowWidth(beam::Vec2 startp, beam::Vec2 endp) {
  int start_row = window_[0], end_row = window_[1];
  int start_col = window_[2], end_col = window_[3];
  // this slope actually calculates perpendicular to the line since
  // origin is top left
  float slope = (endp[1] - startp[1]) / (endp[0] - startp[0]);
  if (abs(slope) < 0.9) {
    // calculate width across every row then take max
    double max_distance = 0.0;
    for (int row = start_row; row < end_row; row++) {
      beam::Vec2 p1, p2;
      for (int col = start_col; col < end_col; col++) {
        if (crack_.at<uchar>(row, col) == 255) {
          beam::Vec2 pixel(row, col);
          p1 = pixel;
        }
      }
      for (int col = end_col; col > start_col; col--) {
        if (crack_.at<uchar>(row, col) == 255) {
          beam::Vec2 pixel(row, col);
          p2 = pixel;
        }
      }
      double distance = dm.GetDistance(p1, p2);
      if (distance > max_distance) max_distance = distance;
    }
    return max_distance;
  } else if (abs(slope) > 1.1) {
    double max_distance = 0.0;
    for (int col = start_col; col < end_col; col++) {
      beam::Vec2 p1, p2;
      for (int row = start_row; row < end_row; row++) {
        if (crack_.at<uchar>(row, col) == 255) {
          beam::Vec2 pixel(row, col);
          p1 = pixel;
        }
      }
      for (int row = end_row; row > start_row; row--) {
        if (crack_.at<uchar>(row, col) == 255) {
          beam::Vec2 pixel(row, col);
          p2 = pixel;
        }
      }
      double distance = dm.GetDistance(p1, p2);
      if (distance > max_distance) max_distance = distance;
    }
    return max_distance;
  } else { // average of both vert and horiz for roughly 45 degree angled line
    double max_distance_vert = 0.0;
    for (int col = start_col; col < end_col; col++) {
      beam::Vec2 p1, p2;
      for (int row = start_row; row < end_row; row++) {
        if (crack_.at<uchar>(row, col) == 255) {
          beam::Vec2 pixel(row, col);
          p1 = pixel;
        }
      }
      for (int row = end_row; row > start_row; row--) {
        if (crack_.at<uchar>(row, col) == 255) {
          beam::Vec2 pixel(row, col);
          p2 = pixel;
        }
      }
      double distance = dm.GetDistance(p1, p2);
      if (distance > max_distance_vert) max_distance_vert = distance;
    }
    double max_distance_horiz = 0.0;
    for (int row = start_row; row < end_row; row++) {
      beam::Vec2 p1, p2;
      for (int col = start_col; col < end_col; col++) {
        if (crack_.at<uchar>(row, col) == 255) {
          beam::Vec2 pixel(row, col);
          p1 = pixel;
        }
      }
      for (int col = end_col; col > start_col; col--) {
        if (crack_.at<uchar>(row, col) == 255) {
          beam::Vec2 pixel(row, col);
          p2 = pixel;
        }
      }
      double distance = dm.GetDistance(p1, p2);
      if (distance > max_distance_horiz) max_distance_horiz = distance;
    }
    return (max_distance_vert + max_distance_horiz) / 2;
  }
}

int GetWindowSide(beam::Vec2 endp, std::vector<int> window) {
  int start_row = window[0], end_row = window[1];
  int start_col = window[2], end_col = window[3];
  if (endp[0] == start_row) {
    return TOP;
  } else if (endp[0] == end_row) {
    return BOTTOM;
  } else if (endp[1] == start_col) {
    return LEFT;
  } else if (endp[1] == end_col) {
    return RIGHT;
  }
}

/*

  // initialize camera model and point cloud
  string intrinsics_loc = cur_dir + "F2.json";
  shared_ptr<beam_calibration::CameraModel> camera =
      beam_calibration::CameraModel::LoadJSON(intrinsics_loc);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>("/home/jake/Thesis/pearson.pcd", *cloud);
  TransformCloud(cloud);

  // undistort and save rgb image
  string image_location = "/home/jake/BGRImage.jpg";
  Mat img = imread(image_location, IMREAD_COLOR);
  img = beam_cv::AdaptiveHistogram(img);
  img = camera->UndistortImage(img);
  Mat img2 = img.clone();
  bilateralFilter(img2, img, 11, 30, 30);
  imwrite("/home/jake/results/image.png", img);

  // initialize and extract depth map
  beam_cv::DepthMap dm(camera, cloud);
  dm.ExtractDepthMap(0.01, 1);
  Mat depth_img = dm.GetDepthImage();

  // perform depth completion

  BEAM_INFO("Performing depth completion...");
  Mat viz = beam_cv::VisualizeDepthImage(depth_img);
  imwrite("/home/jake/results/depth.png", viz);
  Mat dst = beam_cv::MultiscaleInterpolation(depth_img);
  viz = beam_cv::VisualizeDepthImage(dst);
  imwrite("/home/jake/results/depth_combined.png", viz);*/