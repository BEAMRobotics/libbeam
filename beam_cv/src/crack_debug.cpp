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

// functions
std::vector<cv::Mat> SegmentMultiscale(cv::Mat depth_image);
beam::Mat4 InterpolateTransformationMatrices(beam::Mat4 C1, beam::Mat4 C2,
                                             float alpha);
// global variables
string cur_dir = "/home/jake/projects/beam_robotics/libbeam/beam_cv/"
                 "tests/test_data/";
shared_ptr<beam_calibration::CameraModel> F1;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

int main() {
  /*
   * 1. Load point cloud, camera model, extrinsics, and image
   * 2. Transform point cloud to pose and to camera pose
   * 3. Read in crack mask
   * 4. Extract skeleton of crack mask
   * 5. Extract depth map
   * 6. Perform depth completion
   * 7. Calculate area of skeleton by finding the area of each pixel in it
   *    a. if no area exists, then assume its the same as the last pixel
   * 8. calculate area of entire mask by using the scale of the skeleton
   *    a. find closest pixel in the skeleton and use its area
   * 9. Find width by dividing total area by length
   */
  /*
  string intrinsics_loc = cur_dir + "F2.json";
  F1 = beam_calibration::CameraModel::LoadJSON(intrinsics_loc);
  pcl::io::loadPCDFile<pcl::PointXYZ>("/home/jake/Thesis/pearson.pcd", *cloud);
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

  string image_location = "/home/jake/BGRImage.jpg";
  Mat img = imread(image_location, IMREAD_COLOR);
  img = beam_cv::AdaptiveHistogram(img);
  img = F1->UndistortImage(img);
  Mat img2 = img.clone();
  bilateralFilter(img2, img, 11, 30, 30);
  imwrite("/home/jake/results/image.png", img);

  beam_cv::DepthMap dm(F1, transformed_cloud2);
  Mat viz;
  dm.ExtractDepthMap(0.01, 1);
  Mat depth_map = dm.GetDepthImage();
  viz = beam_cv::VisualizeDepthImage(depth_map);
  imwrite("/home/jake/results/depth.png", viz);

  Mat depth_bw = Mat::zeros(Size(depth_map.cols, depth_map.rows), CV_8UC1);

  float min_depth = 1000, max_depth = 0;
  depth_map.forEach<float>([&](float& distance, const int* position) -> void {
    (void)position;
    if (distance != 0.0) {
      if (distance > max_depth) { max_depth = distance; }
      if (distance < min_depth) { min_depth = distance; }
    }
  });

  int scale = 255 / max_depth;

  depth_map.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance != 0) {
      uint8_t pixel_value = (scale * distance);
      depth_bw.at<uchar>(position[0], position[1]) = 255 - pixel_value;
    }
  });
  imwrite("/home/jake/results/depth_map.png", depth_bw);*/
  Mat depth_img;
  string depth_image_path = "/home/jake/depth_input.png";
  Mat depth = imread(depth_image_path, IMREAD_GRAYSCALE);
  depth.convertTo(depth_img, CV_32F);
  vector<Mat> channels = SegmentMultiscale(depth_img);
  Mat viz = beam_cv::VisualizeDepthImage(depth_img);
  imwrite("/home/jake/results/depth.png", viz);

  cv::Mat dst = channels[0];
  viz = beam_cv::VisualizeDepthImage(dst);
  imwrite("/home/jake/results/channel1.png", viz);
  dst = beam_cv::DepthInterpolation(21, 21, 5, dst);
  dst = beam_cv::DepthInterpolation(15, 15, 5, dst);
  cv::Mat diamondKernel5 = beam::GetEllipseKernel(5);
  diamondKernel5.at<uchar>(1, 0) = 0;
  diamondKernel5.at<uchar>(1, 4) = 0;
  diamondKernel5.at<uchar>(3, 0) = 0;
  diamondKernel5.at<uchar>(3, 4) = 0;
  cv::dilate(dst, dst, diamondKernel5);
  cv::morphologyEx(dst, dst, cv::MORPH_CLOSE, beam::GetFullKernel(5));
  cv::morphologyEx(dst, dst, cv::MORPH_CLOSE, beam::GetEllipseKernel(11));
  viz = beam_cv::VisualizeDepthImage(dst);
  imwrite("/home/jake/results/channel1_interp.png", viz);
}

std::vector<cv::Mat> SegmentMultiscale(cv::Mat depth_image) {
  float min_depth_ = 1000, max_depth_ = 0;
  depth_image.forEach<float>([&](float& distance, const int* position) -> void {
    (void)position;
    if (distance != 0.0) {
      if (distance > max_depth_) { max_depth_ = distance; }
      if (distance < min_depth_) { min_depth_ = distance; }
    }
  });

  float channel_width = (max_depth_ - min_depth_) / 4;
  Mat empty = Mat::zeros(Size(depth_image.cols, depth_image.rows), CV_32FC1);
  Mat channel1 = empty.clone(), channel2 = empty.clone(),
      channel3 = empty.clone(), channel4 = empty.clone();
  depth_image.forEach<float>([&](float& distance, const int* position) {
    if (distance >= min_depth_ && distance < min_depth_ + channel_width) {
      channel1.at<float>(position[0], position[1]) = distance;
    } else if (distance >= min_depth_ + channel_width &&
               distance < min_depth_ + 2 * channel_width) {
      channel2.at<float>(position[0], position[1]) = distance;
    } else if (distance >= min_depth_ + 2 * channel_width &&
               distance < min_depth_ + 3 * channel_width) {
      channel3.at<float>(position[0], position[1]) = distance;
    } else if (distance >= min_depth_ + 3 * channel_width &&
               distance < min_depth_ + 4 * channel_width) {
      channel4.at<float>(position[0], position[1]) = distance;
    }
  });
  std::vector<cv::Mat> segments = {channel1, channel2, channel3, channel4};
  return segments;
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