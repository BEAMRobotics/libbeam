#include <beam_calibration/ConvertCameraModel.h>
#include <beam_calibration/Radtan.h>

namespace beam_calibration {

ConvertCameraModel::ConvertCameraModel(
    const std::shared_ptr<CameraModel>& source_model, int output_width,
    int output_height, const std::shared_ptr<CameraModel>& output_model) {
  output_width_ = output_width;
  output_height_ = output_height;

  // if no output model given, create default
  std::shared_ptr<CameraModel> _output_model;
  if (output_model == nullptr) {
    _output_model = CreateDefaultCameraModel(source_model);
  } else {
    _output_model = output_model;
  }

  // create map from output image to source image
  CreatePixelMap(source_model, _output_model);
}

std::shared_ptr<CameraModel> ConvertCameraModel::CreateDefaultCameraModel(
    const std::shared_ptr<CameraModel>& source_model) {
  if (source_model->GetIntrinsics().size() < 4) {
    throw std::runtime_error{
        "Invalid intrinsics dimension in source camera model. Make sure the "
        "first 4 elements in the source camera model are fx, fy, cx, and cy, "
        "respectively."};
  }

  Eigen::Matrix<double, 8, 1> intrinsics;
  intrinsics(0, 0) = source_model->GetIntrinsics()[0];
  intrinsics(1, 0) = source_model->GetIntrinsics()[1];
  // intrinsics(2, 0) = source_model->GetIntrinsics()[2];
  // intrinsics(3, 0) = source_model->GetIntrinsics()[3];
  // intrinsics(0, 0) = output_width_;
  // intrinsics(1, 0) = output_width_;
  intrinsics(2, 0) = output_width_ / 2;
  intrinsics(3, 0) = output_height_ / 2;
  intrinsics(4, 0) = 0;
  intrinsics(5, 0) = 0;
  intrinsics(6, 0) = 0;
  intrinsics(7, 0) = 0;
  std::shared_ptr<CameraModel> model =
      std::make_shared<Radtan>(output_height_, output_width_, intrinsics);
  return model;
}

void ConvertCameraModel::CreatePixelMap(
    const std::shared_ptr<CameraModel>& source_model,
    const std::shared_ptr<CameraModel>& output_model) {
  // check to make sure integer overflow will not occur
  if (source_model->GetWidth() > std::numeric_limits<int32_t>::max() ||
      source_model->GetHeight() > std::numeric_limits<int32_t>::max()) {
    throw std::invalid_argument{"Input image too large."};
  }

  std::cout << "Input size: [" << source_model->GetHeight() << ", "
            << source_model->GetWidth() << "]\n"
            << "Output size: [" << output_model->GetHeight() << ", "
            << output_model->GetWidth() << "]\n"
            << "Input Intrinsics: [" << source_model->GetIntrinsics()[0] << ", "
            << source_model->GetIntrinsics()[1] << ", "
            << source_model->GetIntrinsics()[2] << ", "
            << source_model->GetIntrinsics()[3] << "]\n"
            << "Output Intrinsics: [" << output_model->GetIntrinsics()[0]
            << ", " << output_model->GetIntrinsics()[1] << ", "
            << output_model->GetIntrinsics()[2] << ", "
            << output_model->GetIntrinsics()[3] << "]\n";

  pixel_map_ = cv::Mat(output_height_, output_width_, CV_32SC2);
  for (int i = 0; i < output_height_; i++) {
    for (int j = 0; j < output_width_; j++) {
      opt<Eigen::Vector3d> point_back_projected =
          output_model->BackProject(Eigen::Vector2i(j, i));
      if (!point_back_projected.has_value()) {
        pixel_map_.at<cv::Vec2i>(i, j).val[0] = -1;
        pixel_map_.at<cv::Vec2i>(i, j).val[1] = -1;
        continue;
      }

      opt<Eigen::Vector2i> point_projected =
          source_model->ProjectPoint(point_back_projected.value());
      if (!point_projected.has_value()) {
        pixel_map_.at<cv::Vec2i>(i, j).val[0] = -1;
        pixel_map_.at<cv::Vec2i>(i, j).val[1] = -1;
        continue;
      }

      pixel_map_.at<cv::Vec2i>(i, j).val[0] = point_projected.value()[0];
      pixel_map_.at<cv::Vec2i>(i, j).val[1] = point_projected.value()[1];
    }
  }
}

} // namespace beam_calibration