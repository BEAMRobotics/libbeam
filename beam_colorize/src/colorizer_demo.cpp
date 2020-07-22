#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <beam_calibration/Radtan.h>
#include <beam_colorize/Projection.h>
#include <beam_colorize/RayTrace.h>
#include <beam_cv/Utils.h>
#include <beam_utils/math.hpp>

pcl::PointCloud<pcl::PointXYZ>::Ptr GetPCD(std::string name) {
  std::string cur_dir = __FILE__;
  cur_dir.erase(cur_dir.end() - 22, cur_dir.end());
  cur_dir += "tests/test_data/";
  std::string pcd_location = cur_dir + name;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_location, *cloud) == -1) {
    BEAM_INFO("Couldn't read pcd file:  {}\n", pcd_location);
  } else {
    BEAM_INFO("Opened file: {}", pcd_location);
  }
  return cloud;
}

cv::Mat GetImage(std::string name) {
  std::string cur_dir = __FILE__;
  cur_dir.erase(cur_dir.end() - 22, cur_dir.end());
  cur_dir += "tests/test_data/";
  std::string image_location = cur_dir + name;
  cv::Mat image;
  image = cv::imread(image_location, CV_LOAD_IMAGE_COLOR);
  if (!image.data) {
    std::cout << "Could not open or find the image" << std::endl;
  } else {
    BEAM_INFO("Opened file: {}", image_location);
  }
  return image;
}

std::shared_ptr<beam_calibration::CameraModel> GetIntrinsics(std::string name) {
  std::string cur_dir = __FILE__;
  cur_dir.erase(cur_dir.end() - 22, cur_dir.end());
  cur_dir += "tests/test_data/";
  std::string intrinsics_location = cur_dir + name;
  std::shared_ptr<beam_calibration::CameraModel> model =
      std::make_shared<beam_calibration::Radtan>(intrinsics_location);
  return model;
}

int main(int argc, char* argv[]) {
  /*if (argc < 2 || argc > 2) {
    std::cout << "Usage: ./beam_colorize_example [raytrace or projection]"
              << std::endl;
  } else if (argc == 2) {
    std::string col_type = std::string(argv[1]);
    if (col_type == "raytrace" || col_type == "projection") {
      // initialize chosen colorizer type
      std::unique_ptr<beam_colorize::Colorizer> colorizer;
      if (col_type == "raytrace") {
        colorizer = beam_colorize::Colorizer::Create(
            beam_colorize::ColorizerType::RAY_TRACE);
      } else if (col_type == "projection") {
        colorizer = beam_colorize::Colorizer::Create(
            beam_colorize::ColorizerType::PROJECTION);
      }
      std::string cur_dir = __FILE__;
      cur_dir.erase(cur_dir.end() - 22, cur_dir.end());
      cur_dir += "tests/test_data/";

      // load point cloud and camera intrinsics
      std::string intrinsics_loc = cur_dir + "camera0.json";
      std::shared_ptr<beam_calibration::CameraModel> model =
          std::make_shared<beam_calibration::Radtan>(intrinsics_loc);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
          new pcl::PointCloud<pcl::PointXYZ>);
      std::string cloud_loc = cur_dir + "test2/259_map.pcd";
      pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_loc, *cloud);

      // load Image
      std::string image_location = cur_dir + "test2/259_mask.jpg";
      cv::Mat image;
      image = cv::imread(image_location, CV_LOAD_IMAGE_COLOR);
      if (!image.data) {
        BEAM_INFO("Could not open or find the image: {}", image_location);
        return -1;
      } else {
        BEAM_INFO("Opened file: {}", image_location);
      }

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(
          new pcl::PointCloud<pcl::PointXYZRGB>);
      // set colorizer values
      bool image_distorted = true;
      colorizer->SetPointCloud(cloud);
      colorizer->SetImage(image);
      colorizer->SetIntrinsics(model);
      colorizer->SetDistortion(image_distorted);
      cloud_colored = colorizer->ColorizePointCloud();

      // visualize colored point cloud
      pcl::visualization::PCLVisualizer::Ptr viewer(
          new pcl::visualization::PCLVisualizer("3D Viewer"));
      viewer->setBackgroundColor(200, 200, 200);
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
          cloud_colored);
      viewer->addPointCloud<pcl::PointXYZRGB>(cloud_colored, rgb, "test cloud");
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "test cloud");
      viewer->addCoordinateSystem(1.0);
      viewer->initCameraParameters();
      while (!viewer->wasStopped()) { viewer->spinOnce(10); }
    } else {
      std::cout << "Usage: ./beam_colorize_example [raytrace or projection]"
                << std::endl;
    }
  }*/

  std::unique_ptr<beam_colorize::Colorizer> colorizer =
      beam_colorize::Colorizer::Create(
          beam_colorize::ColorizerType::PROJECTION);

  std::string cur_dir = __FILE__;
  cur_dir.erase(cur_dir.end() - 22, cur_dir.end());
  cur_dir += "tests/test_data/";

  // load point cloud and camera intrinsics
  std::string intrinsics_loc = cur_dir + "camera0.json";
  std::shared_ptr<beam_calibration::CameraModel> model =
      std::make_shared<beam_calibration::Radtan>(intrinsics_loc);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::string cloud_loc = cur_dir + "test2/259_map.pcd";
  pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_loc, *cloud);

  // load Image
  std::string image_location = cur_dir + "test2/259_mask.jpg";
  cv::Mat image;
  image = cv::imread(image_location, CV_LOAD_IMAGE_COLOR);
  if (!image.data) {
    BEAM_INFO("Could not open or find the image: {}", image_location);
    return -1;
  } else {
    BEAM_INFO("Opened file: {}", image_location);
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  // set colorizer values
  bool image_distorted = true;
  colorizer->SetPointCloud(cloud);
  colorizer->SetImage(image);
  colorizer->SetIntrinsics(model);
  colorizer->SetDistortion(image_distorted);
  cloud_colored = colorizer->ColorizePointCloud();

  int non_red = 0;
  for (int i = 0; i < cloud_colored->points.size(); i++) {
    int r = cloud_colored->points[i].r;
    int g = cloud_colored->points[i].g;
    int b = cloud_colored->points[i].b;
    if (r != 0 && g != 0 && b != 0) {
      if (r != 254) { non_red++; }
    }
  }
  std::cout << non_red << std::endl;
}
