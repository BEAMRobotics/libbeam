#include "beam_calibration/Pinhole.h"
#include "beam_calibration/TfTree.h"
#include "beam_colorize/Projection.h"
#include "beam_utils/math.hpp"
#include <boost/filesystem.hpp>
#include <iostream>
#include <nlohmann/json.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudColor;
typedef pcl::visualization::PCLVisualizer Viz;
typedef pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>
    ColourHandler;

double sensitivity_r = 3, sensitivity_t = 5;
bool update_trans, image_distorted, PI = 3.14159265359;
std::string calibration_to_frame, calibration_from_frame, image_file,
    intrinsics, extrinsics, map_file, map_frame, image_frame, filename,
    output_file_name;
beam::Vec3 trans, rot;
Eigen::Affine3d T_C_Map, T_C_Map_updated, T_C_to, T_to_from, T_to_from_updated,
    T_from_Map;
PointCloud::Ptr cloud_transformed(new PointCloud);
PointCloudColor::Ptr cloud_colored(new PointCloudColor);
PointCloud::Ptr cloud(new PointCloud);

beam_colorize::Projection projector;

std::string GetFilePath(std::string filename) {
  std::string file_location = __FILE__;
  file_location.erase(file_location.end() - 21, file_location.end());
  return file_location + "calibrater_data/" + filename;
}

void PrintIntructions() {
  std::cout
      << "Current sensitivity [trans - mm, rot - deg]: [" << sensitivity_t
      << ", " << sensitivity_r << "]"
      << "\n"
      << "Press up/down arrow keys to increase/decrease translation sensitivity"
      << "\n"
      << "Press right/left arrow keys to increase/decrease rotation sensitivity"
      << "\n"
      << "Press buttons 'a/s/d' to increase translational DOF in x/y/z"
      << "\n"
      << "Press buttons 'z/x/v' to decrease translational DOF in x/y/z"
      << "\n"
      << "Press buttons 'k/l/;' to increase rotational DOF about x/y/z"
      << "\n"
      << "Press buttons 'm/,/.' to decrease rotational DOF about x/y/z"
      << "\n"
      << "Press 'End' buttom to save final transform."
      << "\n";
}

void OutputUpdatedTransform() {
  beam::Mat4 T = T_to_from_updated.matrix();
  double T1 = T(0, 0), T2 = T(0, 1), T3 = T(0, 2), T4 = T(0, 3), T5 = T(1, 0),
         T6 = T(1, 1), T7 = T(1, 2), T8 = T(1, 3), T9 = T(2, 0), T10 = T(2, 1),
         T11 = T(2, 2), T12 = T(2, 3), T13 = T(3, 0), T14 = T(3, 1),
         T15 = T(3, 2), T16 = T(3, 3);

  nlohmann::json J = {{"to_frame", calibration_to_frame},
                      {"from_frame", calibration_from_frame},
                      {"transform",
                       {T1, T2, T3, T4, T5, T6, T7, T8, T9, T10, T11, T12, T13,
                        T14, T15, T16}}};
  std::ofstream file(output_file_name);
  file << std::setw(4) << J << std::endl;
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
                           void* viewer_void) {
  update_trans = false;
  trans.setZero();
  rot.setZero();
  pcl::visualization::PCLVisualizer::Ptr viewer =
      *static_cast<Viz::Ptr*>(viewer_void);
  if (event.getKeySym() == "a" && event.keyDown()) {
    trans(0, 0) = sensitivity_t / 1000;
    update_trans = true;
  } else if (event.getKeySym() == "s" && event.keyDown()) {
    trans(1, 0) = sensitivity_t / 1000;
    update_trans = true;
  } else if (event.getKeySym() == "d" && event.keyDown()) {
    trans(2, 0) = sensitivity_t / 1000;
    update_trans = true;
  } else if (event.getKeySym() == "z" && event.keyDown()) {
    trans(0, 0) = -sensitivity_t / 1000;
    update_trans = true;
  } else if (event.getKeySym() == "x" && event.keyDown()) {
    trans(1, 0) = -sensitivity_t / 1000;
    update_trans = true;
  } else if (event.getKeySym() == "v" && event.keyDown()) {
    trans(2, 0) = -sensitivity_t / 1000;
    update_trans = true;
  } else if (event.getKeySym() == "k" && event.keyDown()) {
    rot(0, 0) = sensitivity_r * PI / 180;
    update_trans = true;
  } else if (event.getKeySym() == "l" && event.keyDown()) {
    rot(1, 0) = sensitivity_r * PI / 180;
    update_trans = true;
  } else if (event.getKeySym() == "semicolon" && event.keyDown()) {
    rot(2, 0) = sensitivity_r * PI / 180;
    update_trans = true;
  } else if (event.getKeySym() == "m" && event.keyDown()) {
    rot(0, 0) = -sensitivity_r * PI / 180;
    update_trans = true;
  } else if (event.getKeySym() == "comma" && event.keyDown()) {
    rot(1, 0) = -sensitivity_r * PI / 180;
    update_trans = true;
  } else if (event.getKeySym() == "period" && event.keyDown()) {
    rot(2, 0) = -sensitivity_r * PI / 180;
    update_trans = true;
  } else if (event.getKeySym() == "Up" && event.keyDown()) {
    sensitivity_t += 0.1;
    PrintIntructions();
    update_trans = false;
  } else if (event.getKeySym() == "Down" && event.keyDown()) {
    sensitivity_t -= 0.1;
    PrintIntructions();
    update_trans = false;
  } else if (event.getKeySym() == "Right" && event.keyDown()) {
    sensitivity_r += 0.1;
    PrintIntructions();
    update_trans = false;
  } else if (event.getKeySym() == "Left" && event.keyDown()) {
    sensitivity_r -= 0.1;
    PrintIntructions();
    update_trans = false;
  } else if (event.getKeySym() == "Home" && event.keyDown()) {
  } else if (event.getKeySym() == "End" && event.keyDown()) {
    OutputUpdatedTransform();
    std::cout << "Saved Transform to: " << output_file_name << "\n";
    std::cout
        << "WARNING: this has overridden any previous files with that name\n";
  }

  if (update_trans) {
    Eigen::Affine3d T_tmp;
    T_tmp.matrix().setIdentity();
    T_tmp.matrix().block(0, 0, 3, 3) = beam::LieAlgebraToR(rot);
    T_tmp.matrix().block(0, 3, 3, 1) = trans;
    T_to_from_updated = T_to_from_updated * T_tmp;
    T_C_Map_updated = T_C_to * T_to_from_updated * T_from_Map;
    pcl::transformPointCloud(*cloud, *cloud_transformed, T_C_Map_updated);
    projector.SetPointCloud(cloud_transformed);
    cloud_colored = projector.ColorizePointCloud();
    ColourHandler rgb(cloud_colored);
    viewer->updatePointCloud<pcl::PointXYZRGB>(cloud_colored, rgb,
                                               "calib cloud");
    PrintIntructions();
  }
  return;
}

int main() {
  // load calibrator config json
  output_file_name = GetFilePath("calibration_output.json");
  filename = GetFilePath("LC_calibrater_config.json");
  std::cout << "filename: " << filename << "\n";
  nlohmann::json J;
  std::ifstream file(filename);
  file >> J;
  calibration_to_frame = J["calibration_to_frame"];
  calibration_from_frame = J["calibration_from_frame"];
  intrinsics = J["intrinsics"];
  extrinsics = J["extrinsics"];
  image_file = J["image_file"];
  image_frame = J["image_frame"];
  image_distorted = J["image_distorted"].get<bool>();
  map_file = J["map_file"];
  map_frame = J["map_frame"];

  // Load calibration tree from json
  beam_calibration::TfTree Tree;
  filename = GetFilePath(extrinsics);
  Tree.LoadJSON(filename);

  // load intrinsics
  std::shared_ptr<beam_calibration::Pinhole> F1 =
      std::make_shared<beam_calibration::Pinhole>();
  filename = GetFilePath(intrinsics);
  F1->LoadJSON(filename);

  // load pcd
  std::string pcd_location = GetFilePath(map_file);
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
  T_to_from = Tree.GetTransform(calibration_to_frame, calibration_from_frame);
  T_from_Map = Tree.GetTransform(calibration_from_frame, map_frame);
  T_C_to = Tree.GetTransform(image_frame, calibration_to_frame);
  T_to_from_updated = T_to_from;

  // colour cloud
  T_C_Map = T_C_to * T_to_from * T_from_Map;
  pcl::transformPointCloud(*cloud, *cloud_transformed, T_C_Map);
  projector.SetPointCloud(cloud_transformed);
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
  viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

  PrintIntructions();
  while (!viewer->wasStopped()) { viewer->spinOnce(10); }
}
