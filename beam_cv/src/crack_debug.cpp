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
beam::Mat4 InterpolateTransformationMatrices(beam::Mat4 C1, beam::Mat4 C2,
                                             float alpha);
void TransformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
// global variables
string cur_dir = "/home/jake/projects/beam_robotics/libbeam/beam_cv/"
                 "tests/test_data/";

// variables
cv::Mat crack_, skeleton_;
beam_cv::DepthMap dm;

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