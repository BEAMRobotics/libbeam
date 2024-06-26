#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <beam_calibration/Radtan.h>
#include <beam_colorize/Projection.h>
#include <beam_colorize/RayTrace.h>
#include <beam_utils/math.h>
#include <beam_utils/time.h>

int main(int argc, char* argv[]) {
  if (argc < 2 || argc > 2) {
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
      std::string cloud_loc = cur_dir + "259_map.pcd";
      pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_loc, *cloud);

      // load Image
      std::string image_location = cur_dir + "259_mask.jpg";
      cv::Mat image;
      image = cv::imread(image_location, cv::IMREAD_COLOR);
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
      colorizer->SetImage(image);
      colorizer->SetIntrinsics(model);
      colorizer->SetDistortion(image_distorted);
      struct timespec t;
      beam::tic(&t);
      cloud_colored = colorizer->ColorizePointCloud(cloud);
      float elapsed = beam::toc(&t);
      BEAM_INFO("Colorizing time elapsed: {}", elapsed);

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
  }
}
