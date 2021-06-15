#include <beam_calibration/ConvertCameraModel.h>
#include <beam_calibration/Radtan.h>

namespace beam_calibration {

ConvertCameraModel::ConvertCameraModel(
    const std::shared_ptr<CameraModel>& source_model,
    const Eigen::Vector2i& source_image_size,
    const Eigen::Vector2i& output_image_size,
    const std::shared_ptr<CameraModel>& output_model) {
  src_height_ = source_image_size[0];
  src_width_ = source_image_size[1];
  out_height_ = output_image_size[0];
  out_width_ = output_image_size[1];
  src_model_width_ = source_model->GetWidth();
  src_model_height_ = source_model->GetHeight();

  // check to make sure integer overflow will not occur
  if (src_width_ > std::numeric_limits<int32_t>::max() ||
      src_height_ > std::numeric_limits<int32_t>::max()) {
    throw std::invalid_argument{"Input image too large."};
  }

  // Check dimensions of src model compared to input images
  if (src_width_ != static_cast<int>(source_model->GetWidth()) ||
      src_height_ != static_cast<int>(source_model->GetHeight())) {
    BEAM_WARN(
        "Source image dimensions are different from the source camera model "
        "dimensions. Make sure the images have not been downsampled. Cropping "
        "is fine.");
  }

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

cv::Mat ConvertCameraModel::UpsampleImage(const cv::Mat& image,
                                          int interpolation_method) {
  cv::Mat image_out(src_model_height_, src_model_width_, image.type());
  cv::resize(image, image_out, image_out.size(), interpolation_method);
  return image_out;
}

cv::Mat ConvertCameraModel::DownsampleImage(const cv::Mat& image,
                                            const Eigen::Vector2i& output_size,
                                            int interpolation_method) {
  cv::Mat image_out(output_size[0], output_size[1], image.type());
  cv::resize(image, image_out, image_out.size(), interpolation_method);
  return image_out;
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
  intrinsics(2, 0) = out_width_ / 2;
  intrinsics(3, 0) = out_height_ / 2;
  intrinsics(4, 0) = 0;
  intrinsics(5, 0) = 0;
  intrinsics(6, 0) = 0;
  intrinsics(7, 0) = 0;
  std::shared_ptr<CameraModel> model =
      std::make_shared<Radtan>(out_height_, out_width_, intrinsics);
  return model;
}

void ConvertCameraModel::CreatePixelMap(
    const std::shared_ptr<CameraModel>& source_model,
    const std::shared_ptr<CameraModel>& output_model) {
  BEAM_INFO("Creating distortion map...");

  pixel_map_ = cv::Mat(out_height_, out_width_, CV_32SC2);
  // take into account the size of the output image relative to the output model
  int u_start = (output_model->GetWidth() - out_width_) / 2;
  int v_start = (output_model->GetHeight() - out_height_) / 2;
  for (int i = 0; i < out_height_; i++) {
    for (int j = 0; j < out_width_; j++) {
      Eigen::Vector3d point_back_projected;
      if (!output_model->BackProject(Eigen::Vector2i(j + u_start, i + v_start),
                                     point_back_projected)) {
        pixel_map_.at<cv::Vec2i>(i, j).val[0] = -1;
        pixel_map_.at<cv::Vec2i>(i, j).val[1] = -1;
        continue;
      }

      Eigen::Vector2d point_projected;
      bool in_image_plane = false;
      if (!source_model->ProjectPoint(point_back_projected, point_projected,
                                      in_image_plane)) {
        pixel_map_.at<cv::Vec2i>(i, j).val[0] = -1;
        pixel_map_.at<cv::Vec2i>(i, j).val[1] = -1;
        continue;
      }
      if (in_image_plane) {
        // take into account the size of the input image relative to the input
        // model
        int new_u =
            point_projected[0] - (source_model->GetWidth() - src_width_) / 2;
        int new_v =
            point_projected[1] - (source_model->GetHeight() - src_height_) / 2;

        if (new_u < 0 || new_v < 0 || new_u > src_width_ ||
            new_v > src_height_) {
          pixel_map_.at<cv::Vec2i>(i, j).val[0] = -1;
          pixel_map_.at<cv::Vec2i>(i, j).val[1] = -1;
          continue;
        }

        pixel_map_.at<cv::Vec2i>(i, j).val[0] = new_u;
        pixel_map_.at<cv::Vec2i>(i, j).val[1] = new_v;
      }
    }
  }
  BEAM_INFO("Done.");
}

} // namespace beam_calibration