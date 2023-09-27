#pragma once

#include <DBoW3/DBoW3.h>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include <beam_cv/descriptors/ORBDescriptor.h>
#include <beam_cv/detectors/ORBDetector.h>
#include <beam_utils/filesystem.h>
#include <beam_utils/optional.h>

using json = nlohmann::json;

const std::string DEFAULT_VOCAB_PATH =
    beam::LibbeamRoot() + "/beam_cv/data/orbvoc.dbow3";

namespace beam_cv {

/**
 * @brief class for image database
 */
class ImageDatabase {
public:
  /**
   * @brief Constructor to initialize with empty database and default params
   */
  ImageDatabase();

  /**
   * @brief Constructor to initialize with empty database
   * @param detector_params parameters to use for the FASTSSC detector
   * @param descriptor_params parameters to use for the orb descriptor
   */
  ImageDatabase(const beam_cv::ORBDetector::Params& detector_params,
                const beam_cv::ORBDescriptor::Params& descriptor_params);
  /**
   * @brief Constructor to initialize with a custom vocabulary
   * @param voc custom vocabulary to use
   * @param timestamps_file_path path to timestamp to Result id json file
   */
  ImageDatabase(const beam_cv::ORBDetector::Params& detector_params,
                const beam_cv::ORBDescriptor::Params& descriptor_params,
                const DBoW3::Vocabulary& voc);

  /**
   * @brief Constructor to initialize with already created dbow db
   * @param dbow_file_path path to database (dbow3) file
   * @param timestamps_file_path path to timestamp to Result id json file
   */
  ImageDatabase(const std::string& dbow_file_path,
                const std::string& timestamps_file_path);

  /**
   * @brief Constructor to initialize with already created dbow db
   * @param detector_params parameters to use for the FASTSSC detector
   * @param descriptor_params parameters to use for the orb descriptor
   * @param dbow_file_path path to database (dbow3) file
   * @param timestamps_file_path path to timestamp to Result id json file
   */
  ImageDatabase(const beam_cv::ORBDetector::Params& detector_params,
                const beam_cv::ORBDescriptor::Params& descriptor_params,
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
  std::vector<DBoW3::Result> QueryDatabaseWithImage(const cv::Mat& query_image,
                                                    int N = 2);

  /**
   * @brief Return list of N image id's best matching query image
   * @param features extracted orb features from an image
   */
  std::vector<DBoW3::Result> QueryDatabaseWithFeatures(const cv::Mat& features,
                                                       int N = 2);

  /**
   * @brief Add an image to the database
   */
  void AddImage(const cv::Mat& image, const ros::Time& timestamp);

  /**
   * @brief Gets the timestamp associated to image with index in the database
   */
  beam::opt<ros::Time> GetImageTimestamp(const DBoW3::EntryId& entry_id);

private:
  json index_to_timestamp_map_;
  std::shared_ptr<DBoW3::Database> dbow_db_;
  beam_cv::ORBDescriptor descriptor_;
  beam_cv::ORBDetector detector_;
};

} // namespace beam_cv