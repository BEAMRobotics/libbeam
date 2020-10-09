#include <opencv/cv.h>

#include <beam_calibration/Radtan.h>
#include <beam_cv/Raycast.h>
#include <beam_depth/DepthMap.h>
#include <beam_depth/Utils.h>
#include <beam_utils/utils.hpp>
#include <pcl/io/pcd_io.h>

using namespace cv;
using namespace std;

void HitBehaviour2(std::shared_ptr<cv::Mat> image,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                   const int* position, int index) {
  pcl::PointXYZ origin(0, 0, 0);
  image->at<float>(position[0], position[1]) =
      beam::distance(cloud->points[index], origin);
}

int main() {
  // file locations
  std::string cur_location = __FILE__;
  cur_location.erase(cur_location.end() - 23, cur_location.end());
  cur_location += "beam_depth/tests/test_data/";
  std::string intrinsics_loc = cur_location + "F2.json";
  // load other objects
  std::shared_ptr<beam_calibration::CameraModel> F2 =
      std::make_shared<beam_calibration::Radtan>(intrinsics_loc);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>(cur_location + "259_map.pcd", *cloud);
  // test method exception throwing

  std::shared_ptr<cv::Mat> depth_image =
      std::make_shared<cv::Mat>(F2->GetHeight(), F2->GetWidth(), CV_32FC1);

  /*
beam_depth::DepthMap dm(F2, cloud);
dm.ExtractDepthMap(0.1, 1);
cv::Mat depth = dm.GetDepthImage();
cv::Mat vis = beam_depth::VisualizeDepthImage(depth);
cv::imwrite("/home/jake/depth.jpg", vis);*/

  // make raycasting object
  beam_cv::Raycast<pcl::PointXYZ> caster(cloud, F2, depth_image);

  // perform ray casting of cloud to create depth_image_
  caster.Execute(0.1, HitBehaviour2);

  cv::Mat vis = beam_depth::VisualizeDepthImage(*depth_image);
  cv::imwrite("/home/jake/depth.jpg", vis);
}
