/** @file
 * @ingroup containers
 * Image data container for bridge type inspections
 *
 * @defgroup containers
 * Custom data containers for inspection specific objects. E.g., Image
 * containers with defect masks, special point types for pcl point clouds.
 */

#pragma once

#include <beam/utils/time.hpp>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <string>

namespace beam_containers {
/** @addtogroup containers
 *  @{ */

/**
 * @brief class for bridge type image container
 */
class ImageBridge {
public:
  /**
   * @brief Default constructor
   */
  ImageBridge() = default;

  /**
   * @brief Default destructor
   */
  virtual ~ImageBridge() = default;

  /**
   * @brief Method for setting the BGR image to the container
   * @param bgr_image Image with standard b-g-r color fields, no processing done
   */
  void SetBGRImage(cv::Mat& bgr_image) {
    bgr_image_ = bgr_image;
    is_bgr_image_set_ = true;
  }

  /**
   * @brief Method for getting the BGR image in the container
   * @return bgr_image_ Image with standard b-g-r color fields, no processing
   * done
   */
  cv::Mat GetBGRImage() { return bgr_image_; }

  /**
   * @brief Method for setting the BGR mask to the container
   * @param bgr_mask Image mask with one "grayscale" field. For this class,
   * values of 0, 1, 2 and 3 correspond to no defect, crack, spall and
   * corrosion staining, respectively.
   */
  void SetBGRMask(cv::Mat& bgr_mask) {
    bgr_mask_ = bgr_mask;
    is_bgr_mask_set_ = true;
  }

  /**
   * @brief Method for getting the BGR mask in the container
   * @return bgr_mask_ Image mask with one "grayscale" field.
   */
  cv::Mat GetBGRMask() { return bgr_mask_; }

  /**
   * @brief Method for setting the IR image to the container
   * @param ir_image Infrared image with standard grayscale color field.
   */
  void SetIRImage(cv::Mat& ir_image) {
    ir_image_ = ir_image;
    is_ir_image_set_ = true;
  }

  /**
   * @brief Method for getting the IR image in the container
   * @return ir_image_ Infrared image with standard grayscale color field.
   */
  cv::Mat GetIRImage() { return ir_image_; }

  /**
   * @brief Method for setting the IR mask to the container
   * @param ir_mask Image with standard grayscale color fields. For this class,
   * values of 0 and 1 correspond to sound concrete, and delaminated concrete,
   * respectively.
   */
  void SetIRMask(cv::Mat& ir_mask) {
    ir_mask_ = ir_mask;
    is_ir_mask_set_ = true;
  }

  /**
   * @brief Method for getting the IR mask in the container
   * @return ir_mask_ Image with standard grayscale color fields.
   */
  cv::Mat GetIRMask() { return ir_image_; }

  /**
   * @brief Method for setting the BRG mask method
   * @param bgr_mask_method String with the method name that was used to create
   * the mask.
   */
  void SetBRGMaskMethod(std::string& bgr_mask_method) {
    bgr_mask_method_ = bgr_mask_method;
  }

  /**
   * @brief Method for getting the BRG mask method
   * @return bgr_mask_method_ String with the method name that was used to
   * create the mask.
   */
  std::string GetBRGMaskMethod() { return bgr_mask_method_; }

  /**
   * @brief Method for setting the IR mask method
   * @param ir_mask_method String with the method name that was used to create
   * the mask.
   */
  void SetIRMaskMethod(std::string& ir_mask_method) {
    ir_mask_method_ = ir_mask_method;
  }

  /**
   * @brief Method for getting the IR mask method
   * @return ir_mask_method_ String with the method name that was used to
   * create the mask.
   */
  std::string GetIRMaskMethod() { return ir_mask_method_; }

  /**
   * @brief Method for setting the name of the bag the images were extracted
   * from
   * @param bag_name
   */
  void SetBagName(std::string& bag_name) { bag_name_ = bag_name; }

  /**
   * @brief Method for getting the name of the bag the images were extracted
   * from
   * @return bag_name_
   */
  std::string GetBagName() { return bag_name_; }

  /**
   * @brief Method for setting the time stamp associated with the image
   * @param time_stamp
   */
  void SetTimePoint(beam::TimePoint& time_stamp) { time_stamp_ = time_stamp; }

  /**
   * @brief Method for getting the time stamp associated with the image
   * @return time_stamp_
   */
  beam::TimePoint GetTimePoint() { return time_stamp_; }

  /**
   * @brief Method for setting the image sequence
   * @param image_seq
   */
  void SetImageSeq(int& image_seq) { image_seq_ = image_seq; }

  /**
   * @brief Method for getting the image sequence
   * @return image_seq_
   */
  int GetImageSeq() { return image_seq_; }

  /**
   * @brief Method for setting whether or not the input bgr image is distorted
   * or undistorted
   * @param bgr_is_distorted Default is false.
   */
  void SetBGRIsDistorted(bool bgr_is_distorted) {
    bgr_is_distorted_ = bgr_is_distorted;
  }

  /**
   * @brief Method for getting whether or not the input bgr image is distorted
   * or undistorted
   * @return bgr_is_distorted_
   */
  bool GetBGRIsDistorted() { return bgr_is_distorted_; }

  /**
   * @brief Method for setting whether or not the input ir image is distorted
   * or undistorted
   * @param ir_is_distorted Default is false.
   */
  void SetIRIsDistorted(bool ir_is_distorted) {
    ir_is_distorted_ = ir_is_distorted;
  }

  /**
   * @brief Method for getting whether or not the input ir image is distorted
   * or undistorted
   * @return ir_is_distorted_
   */
  bool GetIRIsDistorted() { return ir_is_distorted_; }

  /**
   * @brief Method for writing all the data inside this container
   * @param output_directory absolute path to output directory
   */
  void Write(std::string output_directory) {
    if (is_bgr_image_set_) {
      cv::imwrite(output_directory + "/BGRImage.jpg", bgr_image_);
    }
    if (is_bgr_mask_set_) {
      cv::imwrite(output_directory + "/BGRMask.jpg", bgr_mask_);
    }
    if (is_ir_image_set_) {
      cv::imwrite(output_directory + "/IRImage.jpg", ir_image_);
    }
    if (is_ir_mask_set_) {
      cv::imwrite(output_directory + "/IRMask.jpg", ir_mask_);
    }

    nlohmann::json J = {{"image_container_type:", "ImageBridge"},
                        {"bgr_mask_method:", bgr_mask_method_},
                        {"ir_mask_method:", ir_mask_method_},
                        {"bag_name:", bag_name_},
                        {"time_stamp:", time_stamp_.time_since_epoch().count()},
                        {"image_seq:", image_seq_},
                        {"bgr_is_distorted:", bgr_is_distorted_},
                        {"ir_is_distorted:", ir_is_distorted_},
                        {"is_bgr_image_set:", is_bgr_image_set_},
                        {"is_bgr_mask_set:", is_bgr_mask_set_},
                        {"is_ir_image_set:", is_ir_image_set_},
                        {"is_ir_mask_set:", is_ir_mask_set_}};

    std::ofstream file(output_directory + "/ImageBridgeInfo.json");
    file << std::setw(4) << J << std::endl;
  }

private:
  cv::Mat bgr_image_, bgr_mask_, ir_image_, ir_mask_;
  std::string bgr_mask_method_, ir_mask_method_, bag_name_;
  beam::TimePoint time_stamp_;
  int image_seq_;
  bool bgr_is_distorted_ = false, ir_is_distorted_ = false,
       is_bgr_image_set_ = false, is_bgr_mask_set_ = false,
       is_ir_image_set_ = false, is_ir_mask_set_ = false;
};

/** @} group containers */

} // namespace beam_containers
