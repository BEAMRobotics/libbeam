/** @file
 * @ingroup calibration
 */

#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <beam_calibration/CameraModel.h>

namespace beam_calibration {

/**
 * @brief This class takes any image taken from some camera model and transforms
 * it to another image fitting a new camera model. The default case is the
 * RADTAN model with no distortion and same image size as the input image. This
 * essential undistorts the image.
 * NOTE: This requires that the first 4 elements in the source camera model to
 * be fx, fy, cx, and cy, respectively.
 */
class ConvertCameraModel {
public:
  /**
   * @brief Constructor
   * @param source_model camera model of source images that will be transformed
   * @param output_width width of output image
   * @param output_height height of output image
   * @param output_model camera model that output images will be transformed to.
   * If this is not specified, it will essentially undistort the image by use a
   * RADTAN model with the same focal length and optical center as the source
   * model but all distortion parameters set to zero
   */
  ConvertCameraModel(
      const std::shared_ptr<CameraModel>& source_model, int output_width,
      int output_height,
      const std::shared_ptr<CameraModel>& output_model = nullptr);

  /**
   * @brief Converts an image taken with the source model to an image with the
   * output_model
   * @param source_image image to be converted
   * @return output image of size output_height x output_width
   */
  template <typename pointT>
  cv::Mat ConvertImage(const cv::Mat& source_image) {
    cv::Mat image_out(output_height_, output_width_, source_image.type());
    for (int i = 0; i < output_height_; i++) {
      for (int j = 0; j < output_width_; j++) {
        int new_u = pixel_map_.at<cv::Vec2i>(i, j).val[0];
        int new_v = pixel_map_.at<cv::Vec2i>(i, j).val[1];
        pointT new_pixel_vals;
        if (new_u < 0 || new_v < 0 || new_u > source_image.cols ||
            new_v > source_image.cols) {
          new_pixel_vals = pointT();
        } else {
          new_pixel_vals = source_image.at<pointT>(new_v, new_u);
        }
        image_out.at<pointT>(i, j) = new_pixel_vals;
      }
    }
    return image_out;
  }

private:
  /**
   * @brief create a RADTAN model with the same focal length and optical center
   * as the source model but all distortion parameters set to zero
   * @param source_model camera model of source images to be converted
   * @return output_model output Radtan model
   */
  std::shared_ptr<CameraModel> CreateDefaultCameraModel(
      const std::shared_ptr<CameraModel>& source_model);

  /**
   * @brief This creates a map of dimensions equal to the output image
   * dimensions, where each element in the map points to the pixel coordinates
   * in the source image to copy to the new image. It sets the member variable
   * pixel_map_
   * @param source_model camera model of source images to be converted
   * @param output_model camera model that output images will be transformed to
   */
  void CreatePixelMap(const std::shared_ptr<CameraModel>& source_model,
                      const std::shared_ptr<CameraModel>& output_model);

  cv::Mat pixel_map_;
  int output_width_;
  int output_height_;
};

} // namespace beam_calibration