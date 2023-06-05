#include <beam_cv/ImageDatabase.h>

namespace beam_cv {

ImageDatabase::ImageDatabase() {
  // construct default detector and descriptor
  beam_cv::ORBDetector::Params detector_params;
  detector_params.num_features = 1000;
  beam_cv::ORBDescriptor::Params descriptor_params;
  detector_ = beam_cv::ORBDetector(detector_params);
  descriptor_ = beam_cv::ORBDescriptor(descriptor_params);
  // construct database
  DBoW3::Vocabulary default_vocab(DEFAULT_VOCAB_PATH);
  dbow_db_ = std::make_shared<DBoW3::Database>(default_vocab);
  index_to_timestamp_map_ = json::object();
}

ImageDatabase::ImageDatabase(
    const beam_cv::ORBDetector::Params& detector_params,
    const beam_cv::ORBDescriptor::Params& descriptor_params) {
  // construct detector and descriptor
  detector_ = beam_cv::ORBDetector(detector_params);
  descriptor_ = beam_cv::ORBDescriptor(descriptor_params);
  // construct database
  DBoW3::Vocabulary default_vocab(DEFAULT_VOCAB_PATH);
  dbow_db_ = std::make_shared<DBoW3::Database>(default_vocab);
  index_to_timestamp_map_ = json::object();
}

ImageDatabase::ImageDatabase(const std::string& dbow_file_path,
                             const std::string& timestamps_file_path) {
  // construct detector and descriptor
  beam_cv::ORBDetector::Params detector_params;
  beam_cv::ORBDescriptor::Params descriptor_params;
  detector_ = beam_cv::ORBDetector(detector_params);
  descriptor_ = beam_cv::ORBDescriptor(descriptor_params);
  // construct database
  dbow_db_ = std::make_shared<DBoW3::Database>(dbow_file_path);
  std::ifstream timestamps_file(timestamps_file_path);
  index_to_timestamp_map_ = json::object();
  timestamps_file >> index_to_timestamp_map_;
}

ImageDatabase::ImageDatabase(
    const beam_cv::ORBDetector::Params& detector_params,
    const beam_cv::ORBDescriptor::Params& descriptor_params,
    const std::string& dbow_file_path,
    const std::string& timestamps_file_path) {
  // construct detector and descriptor
  detector_ = beam_cv::ORBDetector(detector_params);
  descriptor_ = beam_cv::ORBDescriptor(descriptor_params);
  // construct database
  dbow_db_ = std::make_shared<DBoW3::Database>(dbow_file_path);
  std::ifstream timestamps_file(timestamps_file_path);
  index_to_timestamp_map_ = json::object();
  timestamps_file >> index_to_timestamp_map_;
}

void ImageDatabase::SaveDatabase(const std::string& dbow_file_path,
                                 const std::string& timestamps_file_path) {
  dbow_db_->save(dbow_file_path);
  std::ofstream timestamps_file(timestamps_file_path);
  timestamps_file << std::setw(4) << index_to_timestamp_map_ << std::endl;
}

cv::Mat ImageDatabase::GetWord(const cv::Mat& descriptor) {
  auto vocab = dbow_db_->getVocabulary();
  auto word_id = vocab->transform(descriptor);
  return vocab->getWord(word_id);
}

std::vector<DBoW3::Result>
    ImageDatabase::QueryDatabase(const cv::Mat& query_image, int N) {
  std::vector<cv::KeyPoint> kps = detector_.DetectFeatures(query_image);
  cv::Mat features = descriptor_.ExtractDescriptors(query_image, kps);
  DBoW3::QueryResults results;
  dbow_db_->query(features, results, N);
  return results;
}

void ImageDatabase::AddImage(const cv::Mat& image, const ros::Time& timestamp) {
  std::vector<cv::KeyPoint> kps = detector_.DetectFeatures(image);
  cv::Mat features = descriptor_.ExtractDescriptors(image, kps);
  DBoW3::EntryId idx = dbow_db_->add(features);
  index_to_timestamp_map_[std::to_string(idx)] = timestamp.toSec();
}

beam::opt<ros::Time>
    ImageDatabase::GetImageTimestamp(const DBoW3::EntryId& entry_id) {
  std::string index_str = std::to_string(entry_id);
  if (!index_to_timestamp_map_.contains(index_str)) { return {}; }
  double time = index_to_timestamp_map_[std::to_string(entry_id)];
  return ros::Time(time);
}

} // namespace beam_cv
