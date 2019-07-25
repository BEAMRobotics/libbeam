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
  /* std::string file = "/home/jake/Downloads/test.ply";
  fillPosesVector(file);*/
}

void TestDepthMap() {
  std::string cur_dir = "/home/jake/projects/beam_robotics/libbeam/beam_cv";
  // load intrinsics
  std::string intrinsics_location = cur_dir + "/tests/test_data/F1.json";
  std::shared_ptr<beam_calibration::CameraModel> F1 =
      beam_calibration::CameraModel::LoadJSON(intrinsics_location);
  // load Image
  std::string image_location = cur_dir + "/tests/test_data/test.jpg";
  cv::Mat image = cv::imread(image_location, CV_LOAD_IMAGE_COLOR);
  // load pcd
  std::string pcd_location = cur_dir + "/tests/test_data/test.pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_location, *cloud);

  /*
   * Test depth map filling
   */
  beam_cv::DepthMap dm(F1, cloud);
  cv::Mat di_viz;
  // Extract depth image, visualize, save
  cv::Mat kernel = beam::GetCrossKernel(5);
  cv::Mat kernel2 = beam::GetEllipseKernel(5);
  // extract depth map with pixel threshold of 0.1 cm, 0 dilation, and a hit
  // mask of size 3
  /*
  dm.ExtractDepthMap(0.001, 3);
  di_viz = dm.VisualizeDepthImage();
  cv::imwrite("/home/jake/ext.png", di_viz);
  dm.DepthInterpolation(70, 5, 0.05, 2);
  di_viz = dm.VisualizeDepthImage();
  cv::imwrite("/home/jake/interp.png", di_viz);
  pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud = dm.ExtractPointCloud(); */
  /*
   * Test with newly created point cloud

  beam_cv::DepthMap dm2(F1, new_cloud, image);
  dm2.ExtractDepthMap(0.02, 21);
  di_viz = dm2.VisualizeDepthImage();
  cv::imwrite("/home/jake/ext2.png", di_viz);
  dm2.DepthCompletion(kernel);
  di_viz = dm2.VisualizeDepthImage();
  cv::imwrite("/home/jake/completed.png", di_viz);
  dm2.VerticalDepthExtrapolation();
  di_viz = dm2.VisualizeDepthImage();
  cv::imwrite("/home/jake/extrapolated.png", di_viz);  */

  beam_cv::DepthMap dm3(F1, cloud);
  dm3.ExtractDepthMap(0.02, 31);
  di_viz = dm3.VisualizeDepthImage();
  cv::imwrite("/home/jake/extracted.png", di_viz);
  dm3.DepthCompletion(kernel);
  di_viz = dm3.VisualizeDepthImage();
  cv::imwrite("/home/jake/full.png", di_viz);
}

int TestCrackCalculation(int argc, char** argv) {
  if (argc != 3) {
    cout << " Usage: imskeleton ImageToLoadAndDisplay BinaryThreshold" << endl;
    return -1;
  }
  Mat image = imread(argv[1], 0); // Read the file
  if (!image.data)                // Check for invalid input
  {
    cout << "Could not open or find the image" << std::endl;
    return -1;
  }

  // Get theshold of input
  int thresh;
  std::stringstream ss(argv[2]);
  ss >> thresh;

  // process image used (just temp with example image)
  cv::threshold(image, image, thresh, 255, cv::THRESH_BINARY);
  // Initialize skel image and temp storage

  image = beam_cv::RemoveClusters(image, 100);
  cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT, Size(3, 3));
  morphologyEx(image, image, cv::MORPH_CLOSE, element2);
  cv::Mat skeleton = beam_cv::ExtractSkeleton(image);

  // Create a copy of the binary image

  // Initialize map for widths

  std::vector<cv::Mat> seg_skels = beam_cv::SegmentSkeleton(skeleton, image);
  int i = 0;
  for (cv::Mat skel : seg_skels) {
    std::string name = std::to_string(i) + ".png";
    imwrite("/home/jake/" + name, skel);
    i++;
  }

  /*
  for (int i = 1; i < num_comp; ++i) // 0 is all points not showing crack
  {
    std::cout << "Results for Crack: " << i << std::endl;
    if (crack_map[i].size() > 50) {
      std::cout << "Crack Length: " << crack_map[i].size() << std::endl;
      double max_width = *max_element(crack_map[i].begin(), crack_map[i].end());
      double min_width = *min_element(crack_map[i].begin(), crack_map[i].end());
      std::cout << "Max: " << max_width << " and Min: " << min_width
                << std::endl;
    } else {
      std::cout << "Not enough data to yield meaningful results" << std::endl;
    };
  }

  applyColorMap(cm_skel, cm_skel, COLORMAP_HOT);

  namedWindow("Original Image", WINDOW_NORMAL);
  namedWindow("Image Skeleton", WINDOW_NORMAL);
  imshow("Original Image", cm_skel);
  imshow("Image Skeleton", skeleton);
  waitKey(0); // Wait for a keystroke in the window
  return 0;*/
}

void fillPosesVector(std::string file_location) {
  std::string delim = " ";
  std::ifstream file(file_location);
  std::string str;
  ros::Time start;
  while (std::getline(file, str)) {
    if (str.substr(0, 11) == "comment UTC") {
      str.erase(0, 26);
      double time = std::stof(str);
      ros::Time temp(time);
      start = temp;
    }
    if (str == "end_header") { break; }
  }
  std::string s;
  while (std::getline(file, s)) {
    size_t pos = 0;
    std::vector<float> vals;
    while ((pos = s.find(delim)) != std::string::npos) {
      float val = std::stof(s.substr(0, pos));
      s.erase(0, pos + delim.length());
      vals.push_back(val);
    }
    float x = vals[0], y = vals[1], z = vals[2];
    float roll = vals[3], pitch = vals[4], yaw = vals[5];
    float time = vals[6];
    ros::Time t(time);
    Eigen::Affine3d TA;
    Eigen::RowVector3d transl;
    transl << x, y, z;
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
    TA.linear() = q.matrix();
    TA.translation() = transl;
    poses.push_back(TA);
    poses_time.push_back(t);
  }
}