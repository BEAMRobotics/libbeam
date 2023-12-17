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
 *
 * NOTE: This requires that the first 4 elements in the source camera model to
 * be fx, fy, cx, and cy, respectively.
 *
 * NOTE: Be careful with image dimensions. If source images are smaller than the
 * soure camera model image dimensions, this class will assume the image has
 * been cropped, with the cropping centered in the image. If the output image
 * dimensions are smaller than the output camera model image dimensions then it
 * will crop the output image, with the cropping centered in the image. IF YOUR
 * INPUT IMAGE HAS BEEN DOWNSAMPLED, THE OUTPUT IMAGE RESULT WILL NOT BE
 * CORRECT. To work with downsampled images, you need to first call
 * UpsampleImage(), then convert to the correct model, and finally call
 * DownsampleImage() to get the original size of the input.
 */
class ConvertCameraModel {
public:
  /**
   * @brief Constructor
   * @param source_model camera model of source images that will be transformed
   * @param source_image_size size of input images [height, wdith]
   * @param output_image_size size of output images [height, wdith]
   * @param output_model camera model that output images will be transformed to.
   * If this is not specified, it will essentially undistort the image by using
   * a RADTAN model with the same focal length as the source model, the optical
   * center is set to the output image center and all distortion parameters are
   * set to zero.
   */
  ConvertCameraModel(
      const std::shared_ptr<CameraModel>& source_model,
      const Eigen::Vector2i& source_image_size,
      const Eigen::Vector2i& output_image_size,
      const std::shared_ptr<CameraModel>& output_model = nullptr);

  /**
   * @brief Converts an image taken with the source model to an image with the
   * output_model
   * @param source_image image to be converted
   * @return output image of size output_height x output_width
   */
  template <typename pointT>
  cv::Mat ConvertImage(const cv::Mat& source_image) {
    // check dimensions are consistent
    if (source_image.cols != src_width_ || source_image.rows != src_height_) {
      BEAM_ERROR(
          "Invalid source image dimensions. Required: {} x {}, given: {} x {}",
          src_height_, src_width_, source_image.rows, source_image.cols);
      throw std::runtime_error{"Invalid source image dimensions."};
    }

    cv::Mat image_out(out_height_, out_width_, source_image.type());
    for (int i = 0; i < out_height_; i++) {
      for (int j = 0; j < out_width_; j++) {
        int new_u = pixel_map_.at<cv::Vec2i>(i, j).val[0];
        int new_v = pixel_map_.at<cv::Vec2i>(i, j).val[1];
        pointT new_pixel_vals;
        if (new_u < 0 || new_v < 0 || new_u > source_image.cols ||
            new_v > source_image.rows) {
          new_pixel_vals = pointT();
        } else {
          new_pixel_vals = source_image.at<pointT>(new_v, new_u);
        }
        image_out.at<pointT>(i, j) = new_pixel_vals;
      }
    }
    return image_out;
  }

  /**
   * @brief Upsamples the input image to the dimensions of the source camera
   * model. This is useful if converting images produced from a source model
   * that have been downsampled in some other process.
   * @param image image to be upsampled
   * @param interpolation_method interpolation method used. See opencv resize
   * function. Default is bilinear interpolation
   * @return output image of dimensions equal to the source model dimensions
   */
  cv::Mat UpsampleImage(const cv::Mat& image,
                        int interpolation_method = cv::INTER_LINEAR);

  /**
   * @brief Downsamples the image.
   * This is useful if converting images produced from a source model
   * that have been downsampled in some other process. You will first
   * need to upsample these images, run the conversion, then if you want to
   * original input image size, you need to call this downsample function.
   * @param image image to be downsampled
   * @param output_size size to downsample image to. Usually you want to set
   * this as the size of your image before upsampling
   * @param interpolation_method interpolation method used. See opencv resize
   * function. Default is bilinear interpolation
   * @return output image
   */
  cv::Mat DownsampleImage(const cv::Mat& image,
                          const Eigen::Vector2i& output_size,
                          int interpolation_method = cv::INTER_LINEAR);

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
  int src_height_;
  int src_width_;
  int src_model_height_;
  int src_model_width_;
  int out_height_;
  int out_width_;
};

/**
 * @brief This is a wrapper around ConvertCameraModel for the case where we want
 * to undistort images which is equivalent to converting to a radtan model with
 * no distortion. The goal of this wrapper is simply to make the user interface
 * more clear.
 *
 * NOTE: This undistortion function converts the images to a RADTAN model with
 * the same focal length as the source model, the optical center is set to the
 * output image center and all distortion parameters are set to zero. If you
 * want to change the intrinsics, use the base class (ConvertCameraModel)
 *
 * NOTE: Be careful with image dimensions. If source images are smaller than the
 * soure camera model image dimensions, this class will assume the image has
 * been cropped, with the cropping centered in the image. If the output image
 * dimensions are smaller than the output camera model image dimensions then it
 * will crop the output image, with the cropping centered in the image. IF YOUR
 * INPUT IMAGE HAS BEEN DOWNSAMPLED, THE OUTPUT IMAGE RESULT WILL NOT BE
 * CORRECT (use UpsampleImage method on your image before converting it)
 */
class UndistortImages : public ConvertCameraModel {
public:
  UndistortImages(const std::shared_ptr<CameraModel>& source_model,
                  const Eigen::Vector2i& source_image_size,
                  const Eigen::Vector2i& output_image_size)
      : ConvertCameraModel(source_model, source_image_size,
                           output_image_size){};
};

} // namespace beam_calibration