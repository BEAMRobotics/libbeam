#include <opencv/cv.h>

#include <beam_calibration/Radtan.h>
#include <beam_cv/Raycast.h>
#include <beam_depth/DepthMap.h>
#include <beam_depth/Utils.h>
#include <beam_utils/utils.hpp>
#include <pcl/io/pcd_io.h>

using namespace cv;
using namespace std;

int main() {
  // file locations
  std::string intrinsics_loc = "/home/jake/K.json";
  std::string cloud_loc = "/home/jake/example_cloud.pcd";
  // load other objects
  std::shared_ptr<beam_calibration::CameraModel> F1 =
      std::make_shared<beam_calibration::Radtan>(intrinsics_loc);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_loc, *cloud);

  std::shared_ptr<cv::Mat> depth_image =
      std::make_shared<cv::Mat>(F1->GetHeight(), F1->GetWidth(), CV_32FC1);

  // make raycasting object
  beam_cv::Raycast<pcl::PointXYZ> caster(cloud, F1, depth_image);

  // perform ray casting of cloud to create depth_image_
  caster.Execute(0.1,
                 [&](std::shared_ptr<cv::Mat>& image,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                     const int* position, int index) -> void {
                   pcl::PointXYZ origin(0, 0, 0);
                   float d = beam::distance(cloud->points[index], origin);
                   image->at<float>(position[0], position[1]) = d;
                 });

  std::shared_ptr<cv::Mat> image = caster.GetImage();
  cv::Mat vis = beam_depth::VisualizeDepthImage(*image);
  cv::imwrite("/home/jake/depth.jpg", vis);
}
