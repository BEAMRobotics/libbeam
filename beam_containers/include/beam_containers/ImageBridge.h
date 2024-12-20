/** @file
 * @ingroup containers
 * Image data container for bridge type inspections
 *
 * @defgroup containers
 * Custom data containers for inspection specific objects. E.g., Image
 * containers with defect masks, special point types for pcl point clouds.
 */

#pragma once

#include <fstream>

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <beam_utils/filesystem.h>
#include <beam_utils/time.h>

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
   * @brief Method getting whether or not the BGR color image has been set
   * @return is_bgr_image_set_
   */

  bool IsBGRImageSet() const { return is_bgr_image_set_; }

  /**
   * @brief Method getting whether or not the BGR mask has been set
   * @param is_bgr_mask_set_
   */

  bool IsBGRMaskSet() const { return is_bgr_mask_set_; }

  /**
   * @brief Method getting whether or not the IR image has been set
   * @return is_ir_image_set_
   */

  bool IsIRImageSet() const { return is_ir_image_set_; }

  /**
   * @brief Method getting whether or not the IR mask has been set
   * @param is_ir_mask_set_
   */

  bool IsIRMaskSet() const { return is_ir_mask_set_; }

  /**
   * @brief Method for setting the BGR image to the container
   * @param bgr_image Image with standard b-g-r color fields, no processing done
   */
  void SetBGRImage(const cv::Mat& bgr_image) {
    bgr_image_ = bgr_image;
    is_bgr_image_set_ = true;
  }

  /**
   * @brief Method for getting the BGR image in the container
   * @return bgr_image_ Image with standard b-g-r color fields, no processing
   * done
   */
  cv::Mat GetBGRImage() const { return bgr_image_; }

  /**
   * @brief Method for setting the BGR mask to the container
   * @param bgr_mask Image mask with one "grayscale" field. For this class,
   * values of 0, 1, 2, 3 and 4 correspond to no defect, crack, delamination,
   * corrosion and spall, respectively.
   */
  void SetBGRMask(const cv::Mat& bgr_mask) {
    bgr_mask_ = bgr_mask;
    is_bgr_mask_set_ = true;
  }

  /**
   * @brief Method for getting the BGR mask in the container
   * @return bgr_mask_ Image mask with one "grayscale" field.
   */
  cv::Mat GetBGRMask() const { return bgr_mask_; }

  /**
   * @brief Method for setting the IR image to the container
   * @param ir_image Infrared image with standard grayscale color field.
   */
  void SetIRImage(const cv::Mat& ir_image) {
    ir_image_ = ir_image;
    is_ir_image_set_ = true;
  }

  /**
   * @brief Method for getting the IR image in the container
   * @return ir_image_ Infrared image with standard grayscale color field.
   */
  cv::Mat GetIRImage() const { return ir_image_; }

  /**
   * @brief Method for setting the IR mask to the container
   * @param ir_mask Image with standard grayscale color fields. For this class,
   * values of 0 and 1 correspond to sound concrete, and delaminated concrete,
   * respectively.
   */
  void SetIRMask(const cv::Mat& ir_mask) {
    ir_mask_ = ir_mask;
    is_ir_mask_set_ = true;
  }

  /**
   * @brief Method for getting the IR mask in the container
   * @return ir_mask_ Image with standard grayscale color fields.
   */
  cv::Mat GetIRMask() const { return ir_image_; }

  /**
   * @brief Method for setting the BGR mask method
   * @param bgr_mask_method String with the method name that was used to create
   * the mask.
   */
  void SetBGRMaskMethod(const std::string& bgr_mask_method) {
    bgr_mask_method_ = bgr_mask_method;
  }

  /**
   * @brief Method for getting the BGR mask method
   * @return bgr_mask_method_ String with the method name that was used to
   * create the mask.
   */
  std::string GetBGRMaskMethod() const { return bgr_mask_method_; }

  /**
   * @brief Method for setting the IR mask method
   * @param ir_mask_method String with the method name that was used to create
   * the mask.
   */
  void SetIRMaskMethod(const std::string& ir_mask_method) {
    ir_mask_method_ = ir_mask_method;
  }

  /**
   * @brief Method for getting the IR mask method
   * @return ir_mask_method_ String with the method name that was used to
   * create the mask.
   */
  std::string GetIRMaskMethod() const { return ir_mask_method_; }

  /**
   * @brief Method for setting the name of the frame id of the bgr camera
   * @param bgr_frame_id
   */
  void SetBGRFrameId(const std::string& bgr_frame_id) {
    bgr_frame_id_ = bgr_frame_id;
  }

  /**
   * @brief Method for getting the name of the frame id of the bgr camera
   * @return bgr_frame_id_
   */
  std::string GetBGRFrameId() const { return bgr_frame_id_; }

  /**
   * @brief Method for setting the name of the frame id of the IR camera
   * @param bgr_frame_id
   */
  void SetIRFrameId(const std::string& ir_frame_id) {
    ir_frame_id_ = ir_frame_id;
  }

  /**
   * @brief Method for getting the name of the frame id of the IR camera
   * @return bgr_frame_id_
   */
  std::string GetIRFrameId() const { return ir_frame_id_; }

  /**
   * @brief Method for setting the name of the bag the images were extracted
   * from
   * @param bag_name
   */
  void SetBagName(const std::string& bag_name) { bag_name_ = bag_name; }

  /**
   * @brief Method for getting the name of the bag the images were extracted
   * from
   * @return bag_name_
   */
  std::string GetBagName() const { return bag_name_; }

  /**
   * @brief Method for setting the time stamp associated with the image
   * @param time_stamp
   */
  void SetTimePoint(const beam::TimePoint& time_stamp) {
    time_stamp_ = time_stamp;
  }

  /**
   * @brief Method for getting the time stamp associated with the image
   * @return time_stamp_
   */
  beam::TimePoint GetTimePoint() const { return time_stamp_; }

  /**
   * @brief Method for getting the time stamp associated with the image
   * @return time_stamp_
   */
  ros::Time GetRosTime() const { return beam::ChronoToRosTime(time_stamp_); }

  /**
   * @brief Method for setting the image sequence
   * @param image_seq
   */
  void SetImageSeq(int image_seq) { image_seq_ = image_seq; }

  /**
   * @brief Method for getting the image sequence
   * @return image_seq_
   */
  int GetImageSeq() const { return image_seq_; }

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
  bool GetBGRIsDistorted() const { return bgr_is_distorted_; }

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
  bool GetIRIsDistorted() const { return ir_is_distorted_; }

  /**
   * @brief Method for writing all the data inside this container
   * @param output_directory absolute path to output directory
   */
  void Write(const std::string& output_directory) const {
    if (is_bgr_image_set_) {
      cv::imwrite(beam::CombinePaths(output_directory, "BGRImage.jpg"),
                  bgr_image_);
    }
    if (is_bgr_mask_set_) {
      cv::imwrite(beam::CombinePaths(output_directory, "BGRMask.png"),
                  bgr_mask_);
    }
    if (is_ir_image_set_) {
      cv::imwrite(beam::CombinePaths(output_directory, "IRImage.jpg"),
                  ir_image_);
    }
    if (is_ir_mask_set_) {
      cv::imwrite(beam::CombinePaths(output_directory, "IRMask.png"), ir_mask_);
    }

    nlohmann::json J = {{"image_container_type", "ImageBridge"},
                        {"bgr_mask_method", bgr_mask_method_},
                        {"ir_mask_method", ir_mask_method_},
                        {"bgr_frame_id", bgr_frame_id_},
                        {"ir_frame_id", ir_frame_id_},
                        {"bag_name", bag_name_},
                        {"time_stamp", time_stamp_.time_since_epoch().count()},
                        {"image_seq", image_seq_},
                        {"bgr_is_distorted", bgr_is_distorted_},
                        {"ir_is_distorted", ir_is_distorted_},
                        {"is_bgr_image_set", is_bgr_image_set_},
                        {"is_bgr_mask_set", is_bgr_mask_set_},
                        {"is_ir_image_set", is_ir_image_set_},
                        {"is_ir_mask_set", is_ir_mask_set_}};

    std::ofstream file(beam::CombinePaths(output_directory, "ImageInfo.json"));
    file << std::setw(4) << J << std::endl;
  }

  /**
   * @brief Populate container based on JSON file
   * @param path_to_json
   */
  void LoadFromJSON(const std::string& path_to_json) {
    nlohmann::json json_config;
    if (!beam::ReadJson(path_to_json + "/ImageInfo.json", json_config)) {
      throw std::runtime_error{"invalid file path"};
    }

    bag_name_ = json_config["bag_name"];
    bgr_frame_id_ = json_config["bgr_frame_id"];
    bgr_is_distorted_ = json_config["bgr_is_distorted"];
    bgr_mask_method_ = json_config["bgr_mask_method"];
    image_seq_ = json_config["image_seq"];
    ir_frame_id_ = json_config["ir_frame_id"];
    ir_is_distorted_ = json_config["ir_is_distorted"];
    ir_mask_method_ = json_config["ir_mask_method"];
    is_bgr_image_set_ = json_config["is_bgr_image_set"];
    is_bgr_mask_set_ = json_config["is_bgr_mask_set"];
    is_ir_image_set_ = json_config["is_ir_image_set"];
    is_ir_mask_set_ = json_config["is_ir_mask_set"];

    beam::TimePoint tp{std::chrono::duration_cast<beam::TimePoint::duration>(
        std::chrono::nanoseconds(json_config["time_stamp"]))};
    time_stamp_ = tp;

    if (is_bgr_image_set_) {
      cv::Mat bgr_img = cv::imread(path_to_json + "/BGRImage.jpg");
      SetBGRImage(bgr_img);
    }
    if (is_bgr_mask_set_) {
      cv::Mat bgr_mask =
          cv::imread(beam::CombinePaths(path_to_json, "BGRMask.png"),
                     cv::IMREAD_GRAYSCALE);
      SetBGRMask(bgr_mask);
    }
    if (is_ir_image_set_) {
      cv::Mat ir_img =
          cv::imread(beam::CombinePaths(path_to_json, "IRImage.jpg"));
      SetIRImage(ir_img);
    }
    if (is_ir_mask_set_) {
      cv::Mat ir_mask =
          cv::imread(beam::CombinePaths(path_to_json, "IRMask.png"));
      SetIRMask(ir_mask);
    }
  }

private:
  cv::Mat bgr_image_;
  cv::Mat bgr_mask_;
  cv::Mat ir_image_;
  cv::Mat ir_mask_;
  std::string bgr_mask_method_;
  std::string ir_mask_method_;
  std::string bag_name_;
  std::string ir_frame_id_;
  std::string bgr_frame_id_;
  beam::TimePoint time_stamp_;
  int image_seq_;
  bool bgr_is_distorted_{false};
  bool ir_is_distorted_{false};
  bool is_bgr_image_set_{false};
  bool is_bgr_mask_set_{false};
  bool is_ir_image_set_{false};
  bool is_ir_mask_set_{false};
};

/** @} group containers */

} // namespace beam_containers
