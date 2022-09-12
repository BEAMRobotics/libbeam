#pragma once

#include <DBoW3/DBoW3.h>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include <beam_cv/descriptors/ORBDescriptor.h>
#include <beam_cv/detectors/GFTTDetector.h>
#include <beam_utils/filesystem.h>

namespace json = nlohmann::json;

const std::string DEFAULT_VOCAB_PATH =
    beam::LibbeamRoot() + "/beam_cv/data/orbvoc.dbow3";

namespace beam_cv {

/**
 * @brief class for image database
 */
class ImageDatabase {
public:
  /**
   * @brief Constructor to initialize with empty database
   * @param database_path path to database folder
   */
  ImageDatabase(const GFTTDetector::Params& detector_params,
                const ORBDescriptor::Params& descriptor_params);

  /**
   * @brief Constructor to initialize with already created dbow db
   * @param database_path path to database folder
   */
  ImageDatabase(const GFTTDetector::Params& detector_params,
                const ORBDescriptor::Params& descriptor_params,
                const std::string& dbow_file_path,
                const std::string& timestamps_file_path);

  /**
   * @brief Default destructor
   */
  ~ImageDatabase() = default;

  /**
   * @brief Save current database to default folder
   */
  void SaveDatabase(const std::string& dbow_file_path,
                    const std::string& timestamps_file_path);

  /**
   * @brief Gets the word for a given descriptor
   */
  cv::Mat GetWord(const cv::Mat& descriptor);

  /**
   * @brief Return list of N image id's best matching query image
   * @param query_image to query database with
   */
  std::vector<uint64_t> QueryDatabase(const cv::Mat& query_image, int N = 2);

  /**
   * @brief Add an image to the database
   */
  uint64_t AddImage(const cv::Mat& image, const ros::Time& timestamp);

  /**
   * @brief Gets the timestamp associated to image with index in the database
   */
  ros::Time GetImageTimestamp(uint64_t index);

private:
  json index_to_timestamp_map_;
  std::shared_ptr<DBoW3::Database> dbow_db_;
  beam_cv::ORBDescriptor descriptor_;
  beam_cv::GFTTDetector detector_;
};

} // namespace beam_cv