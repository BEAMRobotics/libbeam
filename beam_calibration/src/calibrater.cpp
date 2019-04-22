#include "beam_calibration/TfTree.h"
#include "beam_calibration/Pinhole.h"
#include "beam_colorize/Projection.h"
#include "beam_utils/math.hpp"
#include <boost/filesystem.hpp>
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <nlohmann/json.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudColor;
typedef pcl::visualization::PCLVisualizer Viz;
typedef pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> ColourHandler;

double sensitivity_r = 3, sensitivity_t = 5;
bool update_trans, PI = 3.14159265359;
beam::Vec3 trans, rot;
Eigen::Affine3d TA, TA_updated;

PointCloud::Ptr cloud_transformed(new PointCloud);
PointCloudColor::Ptr cloud_colored(new PointCloudColor);
PointCloud::Ptr cloud(new PointCloud);

beam_colorize::Projection projector;

std::string GetFilePath(std::string filename){
  std::string file_location = __FILE__;
  file_location.erase(file_location.end() - 18, file_location.end());
  return file_location + "tests/test_data/" + filename;
}

void PrintIntructions(){
  std::cout << "Current sensitivity [trans - mm, rot - deg]: [" << sensitivity_t
            << ", " << sensitivity_r << "]"<< "\n"
            << "Input b/n keys to increase/decrease translation sensitivity" << "\n"
            << "Input [ / ] keys to increase/decrease rotation sensitivity" << "\n"
            << "Input buttons 'a/s/d' to increase translational DOF in x/y/z" << "\n"
            << "Input buttons 'z/x/v' to decrease translational DOF in x/y/z" << "\n"
            << "Input buttons 'k/l/;' to increase rotational DOF about x/y/z" << "\n"
            << "Input buttons 'm/,/.' to decrease rotational DOF about x/y/z" << "\n";
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                        void* viewer_void)
{
  update_trans = false;
  trans.setZero();
  rot.setZero();
  pcl::visualization::PCLVisualizer::Ptr viewer = *static_cast<Viz::Ptr *> (viewer_void);
  if (event.getKeySym () == "a" && event.keyDown ())
  {
    trans(0,0) = sensitivity_t/1000;
    update_trans = true;
  } else if (event.getKeySym () == "s" && event.keyDown ()) {
    trans(1,0) = sensitivity_t/1000;
    update_trans = true;
  } else if (event.getKeySym () == "d" && event.keyDown ()) {
    trans(2,0) = sensitivity_t/1000;
    update_trans = true;
  } else if (event.getKeySym () == "z" && event.keyDown ()) {
    trans(0,0) = -sensitivity_t/1000;
    update_trans = true;
  } else if (event.getKeySym () == "x" && event.keyDown ()) {
    trans(1,0) = -sensitivity_t/1000;
    update_trans = true;
  } else if (event.getKeySym () == "v" && event.keyDown ()) {
    trans(2,0) = -sensitivity_t/1000;
    update_trans = true;
  } else if (event.getKeySym () == "k" && event.keyDown ()) {
    rot(0,0) = sensitivity_r*PI/180;
    update_trans = true;
  } else if (event.getKeySym () == "l" && event.keyDown ()) {
    rot(1,0) = sensitivity_r*PI/180;
    update_trans = true;
  } else if (event.getKeySym () == "semicolon" && event.keyDown ()) {
    rot(2,0) = sensitivity_r*PI/180;
    update_trans = true;
  } else if (event.getKeySym () == "m" && event.keyDown ()) {
    rot(0,0) = -sensitivity_r*PI/180;
    update_trans = true;
  } else if (event.getKeySym () == "comma" && event.keyDown ()) {
    rot(1,0) = -sensitivity_r*PI/180;
    update_trans = true;
  } else if (event.getKeySym () == "period" && event.keyDown ()) {
    rot(2,0) = -sensitivity_r*PI/180;
    update_trans = true;
  } else if (event.getKeySym () == "b" && event.keyDown ()) {
    sensitivity_t+=0.1;
    PrintIntructions();
    update_trans = false;
  } else if (event.getKeySym () == "n" && event.keyDown ()) {
    sensitivity_t-=0.1;
    PrintIntructions();
    update_trans = false;
  } else if (event.getKeySym () == "bracketleft" && event.keyDown ()) {
    sensitivity_r+=0.1;
    PrintIntructions();
    update_trans = false;
  } else if (event.getKeySym () == "bracketright" && event.keyDown ()) {
    sensitivity_r-=0.1;
    PrintIntructions();
    update_trans = false;
  }

  if(update_trans){
    Eigen::Affine3d TA_tmp;
    TA_tmp.matrix().setIdentity();
    TA_tmp.matrix().block(0,0,3,3) = beam::LieAlgebraToR(rot);
    TA_tmp.matrix().block(0,3,3,1) = trans;
    TA_updated = TA_updated * TA_tmp;
    pcl::transformPointCloud(*cloud, *cloud_transformed, TA_updated);
    projector.SetPointCloud(cloud_transformed);
    cloud_colored = projector.ColorizePointCloud();
    ColourHandler rgb(cloud_colored);
    viewer->updatePointCloud<pcl::PointXYZRGB>(cloud_colored, rgb, "calib cloud");
    PrintIntructions();
  }
  return;
}

int main() {

  // load calibrator config json
  std::string transform_pts_to_frame, transform_pts_from_frame, image_file,
              intrinsics, extrinsics, map;
  std::string filename = GetFilePath("calibrater_config.json");
  nlohmann::json J;
  std::ifstream file(filename);
  file >> J;
  transform_pts_to_frame = J["transform_pts_to_frame"];
  transform_pts_from_frame = J["transform_pts_from_frame"];
  image_file = J["image"];
  intrinsics = J["intrinsics"];
  extrinsics = J["extrinsics"];
  map = J["map"];

  // Load calibration tree from json
  beam_calibration::TfTree Tree;
  filename = GetFilePath(extrinsics);
  Tree.LoadJSON(filename);

  // load intrinsics
  std::shared_ptr<beam_calibration::Pinhole> F1 = std::make_shared<beam_calibration::Pinhole>();
  filename = GetFilePath(intrinsics);
  F1->LoadJSON(filename);

  // load pcd
  std::string pcd_location = GetFilePath(map);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_location, *cloud) == -1) {
    LOG_INFO("Couldn't read pcd file:  %s\n", pcd_location.c_str());
    return (-1);
  } else {
    LOG_INFO("Opened file: %s", pcd_location.c_str());
  }

  // load Image
  std::string image_location = GetFilePath(image_file);
  cv::Mat image;
  image = cv::imread(image_location, CV_LOAD_IMAGE_COLOR);

  // Get transforms needed
  TA = Tree.GetTransform(transform_pts_to_frame, transform_pts_from_frame);
  TA_updated = TA;

  // colour cloud
  pcl::transformPointCloud(*cloud, *cloud_transformed, TA);
  bool image_distorted = true;
  projector.SetPointCloud(cloud_transformed);
  // projector.SetTransform(TA); //TA_C_L
  projector.SetImage(image);
  projector.SetIntrinsics(F1);
  projector.SetDistortion(image_distorted);
  cloud_colored = projector.ColorizePointCloud();

  // viewer
  Viz::Ptr viewer(new Viz("3D Viewer"));
  viewer->setBackgroundColor(0.8, 0.8, 0.8);
  ColourHandler rgb(cloud_colored);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud_colored, rgb, "calib cloud");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "calib cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);

  PrintIntructions();
  while (!viewer->wasStopped()) {
    viewer->spinOnce(10);
  }

}
