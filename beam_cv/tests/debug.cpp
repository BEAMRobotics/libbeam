#include "beam_calibration/CameraModel.h"
#include "beam_calibration/TfTree.h"
#include "beam_cv/geometry/RelativePoseEstimator.h"
#include "beam_cv/geometry/Triangulation.h"
#include "beam_defects/Defect.h"
#include "beam_defects/Delam.h"
#include "beam_defects/extract_functions.h"
#include "beam_utils/math.hpp"
#include <beam_containers/PointBridge.h>

#include <fstream>
#include <iostream>

#include <pcl/io/pcd_io.h>

using namespace std;

void CalculateGTError(double scale);
void RelabelMask();
void QuantifyDefects();
void TestTriangulation();
void TestEssentialMatrix();

int main(int argc, char* argv[]) {
  // double scale;
  // sscanf(argv[1], "%lf", &scale);
  // CalculateGTError(scale);
  // RelabelMask();
  // QuantifyDefects();
  TestEssentialMatrix();
}

void TestTriangulation() {
  std::string cam_loc = __FILE__;
  cam_loc.erase(cam_loc.end() - 15, cam_loc.end());
  cam_loc += "tests/test_data/K.json";
  std::shared_ptr<beam_calibration::CameraModel> cam =
      beam_calibration::CameraModel::Create(cam_loc);

  Eigen::Matrix4d Pr = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d Pc;
  Pc << 1, 0, 0, -1, 0, 0, 1, -0, 0, -1, 0, -0, 0, 0, 0, 1;
  Eigen::Vector2i pr(1442, 1680);
  Eigen::Vector2i pc(2484, 1845);

  opt<Eigen::Vector3d> pt3d =
      beam_cv::Triangulation::TriangulatePoint(cam, cam, Pr, Pc, pr, pc);
  if (pt3d.has_value()) {
    std::cout << "\nTriangulated point: \n"
              << pt3d.value() << "\n"
              << std::endl;
    Eigen::Vector4d pt3d_h;
    pt3d_h << pt3d.value()[0], pt3d.value()[1], pt3d.value()[2], 1;

    Eigen::Vector4d img1_pt3d_h = Pr * pt3d_h;
    Eigen::Vector3d img1_pt3d = img1_pt3d_h.head(3) / img1_pt3d_h(3);
    opt<Eigen::Vector2i> img1_point_reprojected = cam->ProjectPoint(img1_pt3d);
    std::cout << "\nTransformed into image 1: \n"
              << img1_pt3d << "\n"
              << std::endl;
    if (img1_point_reprojected.has_value()) {
      std::cout << img1_point_reprojected.value() << std::endl;
    }

    Eigen::Vector4d img5_pt3d_h = Pc * pt3d_h;
    Eigen::Vector3d img5_pt3d = img5_pt3d_h.head(3) / img5_pt3d_h(3);
    std::cout << "\nTransformed into image 2: \n"
              << img5_pt3d << "\n"
              << std::endl;
    opt<Eigen::Vector2i> img5_point_reprojected = cam->ProjectPoint(img5_pt3d);
    if (img5_point_reprojected.has_value()) {
      std::cout << img5_point_reprojected.value() << std::endl;
    }
  }
}

void CalculateGTError(double scale) {
  double area = 0;
  cv::Mat mask = cv::imread("/home/jake/mask.jpg", cv::IMREAD_GRAYSCALE);
  for (int i = 0; i < mask.rows; i++) {
    for (int j = 0; j < mask.cols; j++) {
      if (mask.at<uchar>(i, j) > 100) { area += scale; }
    }
  }
  std::cout << area << std::endl;
}

void RelabelMask() {
  cv::Mat mask = cv::imread("/home/jake/mask.jpg", cv::IMREAD_GRAYSCALE);
  for (int i = 0; i < mask.rows; i++) {
    for (int j = 0; j < mask.cols; j++) {
      if (mask.at<uchar>(i, j) > 140) {
        mask.at<uchar>(i, j) = 2;
      } else {
        mask.at<uchar>(i, j) = 0;
      }
    }
  }
  cv::imwrite("/home/jake/mask2.jpg", mask);
}

void QuantifyDefects() {
  pcl::PCDReader reader;
  auto cloud =
      boost::make_shared<pcl::PointCloud<beam_containers::PointBridge>>();
  reader.read("/home/jake/data/IronHorse/clouds/cam0_51_6/subsampled.pcd",
              *cloud);

  float threshold = 0.2;
  std::vector<beam_defects::Defect::Ptr> delam_vector_ =
      beam_defects::GetDelams(cloud, threshold, 0.2, 3, 80000);

  for (auto& defect : delam_vector_) {
    std::cout << "Delam size is: " << defect->GetSize() << "m^2" << std::endl;
  }
}

void TestEssentialMatrix() {
  std::string cam_loc = __FILE__;
  cam_loc.erase(cam_loc.end() - 15, cam_loc.end());
  cam_loc += "tests/test_data/K.json";
  std::shared_ptr<beam_calibration::CameraModel> cam =
      beam_calibration::CameraModel::Create(cam_loc);

  std::string frame1_loc = __FILE__;
  frame1_loc.erase(frame1_loc.end() - 15, frame1_loc.end());
  frame1_loc += "tests/test_data/frame1_matches.txt";
  std::string frame2_loc = __FILE__;
  frame2_loc.erase(frame2_loc.end() - 15, frame2_loc.end());
  frame2_loc += "tests/test_data/frame2_matches.txt";
  // declare variables
  std::ifstream infile;
  std::string line;
  // open file
  infile.open(frame1_loc);
  // extract poses
  std::vector<Eigen::Vector2i> frame1_matches;
  while (!infile.eof()) {
    // get timestamp k
    std::getline(infile, line, ',');
    int u = std::stod(line);
    std::getline(infile, line, '\n');
    int v = std::stod(line);
    Eigen::Vector2i p{u, v};
    frame1_matches.push_back(p);
  }
  // declare variables
  std::ifstream infile2;
  std::string line2;
  // open file
  infile2.open(frame2_loc);
  // extract poses
  std::vector<Eigen::Vector2i> frame2_matches;
  while (!infile2.eof()) {
    // get timestamp k
    std::getline(infile2, line2, ',');
    int u = std::stod(line2);
    std::getline(infile2, line2, '\n');
    int v = std::stod(line2);
    Eigen::Vector2i p{u, v};
    frame2_matches.push_back(p);
  }
  opt<Eigen::Matrix3d> E =
      beam_cv::RelativePoseEstimator::EssentialMatrix8Point(
          cam, cam, frame1_matches, frame2_matches);
  std::cout << E.value() << "\n--------- " << std::endl;
  std::vector<Eigen::Matrix3d> R;
  std::vector<Eigen::Vector3d> t;
  beam_cv::RelativePoseEstimator::RtFromE(E.value(), R, t);
  Eigen::Matrix4d pose = beam_cv::RelativePoseEstimator::RecoverPose(
      cam, cam, frame1_matches, frame2_matches, R, t);
  std::cout << pose << std::endl;
}
