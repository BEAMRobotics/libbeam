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
#define BOTTOM 1
#define TOP 2
#define LEFT 3
#define RIGHT 4

void TestDepthMap();
void TestCrackCalculation();
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

int main(int argc, char* argv[]) {
  if (atoi(argv[1]) == 1) {
    TestCrackCalculation();
  } else if (atoi(argv[1]) == 0) {
    TestDepthMap();
  }
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
  /*
   * Test depth map filling
   */
  dm.SetCloud(cloud);
  dm.SetModel(F1);
  cv::Mat di_viz;

  dm.ExtractDepthMap(0.003, 3);
  di_viz = dm.VisualizeDepthImage();
  cv::imwrite("/home/jake/ext.png", di_viz);
  dm.DepthInterpolation(70, 5, 0.05, 4);
  di_viz = dm.VisualizeDepthImage();
  cv::imwrite("/home/jake/interp.png", di_viz);
}

void TestCrackCalculation() {
  Mat image = imread("/home/jake/Downloads/test2.png", 0);
  std::string cur_dir = "/home/jake/projects/beam_robotics/libbeam/beam_cv";
  // load intrinsics
  std::string intrinsics_location = cur_dir + "/tests/test_data/F1.json";
  std::shared_ptr<beam_calibration::CameraModel> F1 =
      beam_calibration::CameraModel::LoadJSON(intrinsics_location);
  // load pcd
  std::string pcd_location = cur_dir + "/tests/test_data/test.pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_location, *cloud);
  // process image used (just temp with example image)
  cv::threshold(image, image, 0, 255, cv::THRESH_BINARY);
  image = beam_cv::RemoveClusters(image, 100);
  cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT, Size(3, 3));
  morphologyEx(image, image, cv::MORPH_CLOSE, element2);
  std::vector<cv::Mat> cracks = beam_cv::SegmentComponents(image);
  crack_ = cracks[2];
  skeleton_ = beam_cv::ExtractSkeleton(crack_);

  // crate depth map

  dm.SetCloud(cloud);
  dm.SetModel(F1);
  dm.ExtractDepthMap(0.02, 31);
  dm.DepthCompletion(kernel);

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
  cv::imshow("skel", skeleton_);
  cv::waitKey(0);
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