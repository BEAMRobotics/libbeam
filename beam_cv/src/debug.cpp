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
  // TestCrackCalculation(argc, argv);
  TestDepthMap();
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
  beam_cv::DepthMap dm(F1, cloud, image);
  cv::Mat di_viz;
  // Extract depth image, visualize, save
  cv::Mat kernel = beam::GetCrossKernel(5);
  // extract depth map with pixel threshold of 0.1 cm, 0 dilation, and a hit
  // mask of size 3
  dm.ExtractDepthMap(0.001, 0, 3);
  di_viz = dm.VisualizeDepthImage();
  cv::imwrite("/home/jake/ext.png", di_viz);
  dm.DepthInterpolation(70, 10, 0.15, 0, 3);
  di_viz = dm.VisualizeDepthImage();
  cv::imwrite("/home/jake/interp.png", di_viz);
  pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud = dm.ExtractPointCloud();
  /*
   * Test with newly created point cloud
   */
  beam_cv::DepthMap dm2(F1, new_cloud, image);
  dm2.ExtractDepthMap(0.02, 0, 31);
  di_viz = dm2.VisualizeDepthImage();
  cv::imwrite("/home/jake/ext2.png", di_viz);
  dm2.DepthCompletion(kernel);
  di_viz = dm2.VisualizeDepthImage();
  cv::imwrite("/home/jake/comp2.png", di_viz);
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
  image = beam_cv::CloseObjects(image);
  cv::Mat skeleton = beam_cv::ExtractSkeleton(image);

  // Create a copy of the binary image
  cv::Mat image_copy = image.clone();

  cv::Mat im_category(image.size(), CV_16UC1);
  cv::Mat my_stats, my_centroids;
  int connectivity = 8;
  int itype = CV_16U;
  int num_comp = cv::connectedComponentsWithStats(
      image, im_category, my_stats, my_centroids, connectivity, itype);
  im_category.convertTo(im_category, CV_8UC1);

  // Initialize map for widths
  std::map<int, std::vector<double>> crack_map;

  // Calc the length of the crack
  int crack_length = cv::sum(skeleton)[0] / 255;
  std::cout << "Total Length : " << crack_length << std::endl;

  // Calc average width (pixels) by dividing total area by total length
  double avg_width = cv::sum(image_copy)[0] / cv::sum(skeleton)[0];
  std::cout << "Average width: " << avg_width << " pixels." << std::endl;

  // Example forced width calculation
  int resolution = std::nearbyint(std::sqrt(avg_width) * 0.5f) * 2.0f * 5 + 1;
  // int resolution = 9;
  std::cout << "Filter Resolution: " << resolution << std::endl;
  int stepper = (resolution - 1) / 2;
  double scale = std::floor(1.0 / num_comp * 255); // scale for colormap

  // initialize smaller window matrices
  cv::Mat im_orig(resolution, resolution, CV_8UC1);
  cv::Mat im_skel(resolution, resolution, CV_8UC1);
  cv::Mat cm_skel(image.size(), CV_8UC1, cv::Scalar(0));
  for (int x = 0; x < skeleton.rows; ++x) // iterate over skeleton image
  {
    for (int y = 0; y < skeleton.cols; ++y) // iterate over skeleton image
    {
      int crack_num = int(im_category.at<uchar>(x, y));
      cm_skel.at<uchar>(x, y) = crack_num * scale;
      if (skeleton.at<uchar>(x, y) == 255) // if skeleton pixel
      {
        for (int i = 0; i < im_skel.rows; ++i) // iterate over window
        {
          for (int j = 0; j < im_skel.cols; ++j) {
            // populate window with the values from skeleton and binary image
            int idx_1 = x + i - stepper;
            int idx_2 = y + j - stepper;

            im_orig.at<uchar>(i, j) = image_copy.at<uchar>(idx_1, idx_2);
            im_skel.at<uchar>(i, j) = skeleton.at<uchar>(idx_1, idx_2);
          }
        }

        double width = cv::sum(im_orig)[0] / cv::sum(im_skel)[0];
        crack_map[crack_num].push_back(width);
      }
    }
  }

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
  return 0;
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