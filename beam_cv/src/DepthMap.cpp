#include "beam_cv/DepthMap.h"

namespace beam_cv {

DepthMap::DepthMap(cv::Mat input_image,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                   std::shared_ptr<beam_calibration::CameraModel> input_model) {
  *image_ = input_image;
  pcl::copyPointCloud(*input_cloud, *point_cloud_);
  cam_model_ = input_model;
  *depth_image_ = cv::Mat(image_->rows, image_->cols, CV_32FC3);
}

cv::Mat DepthMap::ExtractDepthMap() {
  depth_image_->forEach<cv::Vec3f>(
      [&](cv::Vec3f& pixel, const int* position) -> void {
        int u = position[0], v = position[1];
      });

  // TODO: ray casting to create a depth image
}

} // namespace beam_cv