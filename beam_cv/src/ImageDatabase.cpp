#include <beam_cv/ImageDatabase.h>
#include <beam_utils/optional.h>

namespace beam_cv {

ImageDatabase::ImageDatabase(const GFTTDetector::Params& detector_params,
                             const ORBDescriptor::Params& descriptor_params, ) {
  // construct detector and descriptor
  detector_ = beam_cv::GFTTDetector(detector_params);
  descriptor_ = beam_cv::ORBDescriptor(descriptor_params);
  // construct database
  DBoW3::Vocabulary default_vocab(DEFAULT_VOCAB_PATH);
  dbow_db_ = std::make_shared<DBoW3::Database>(default_vocab);
  index_to_timestamp_map_ = json::object();
}

ImageDatabase::ImageDatabase(const GFTTDetector::Params& detector_params,
                             const ORBDescriptor::Params& descriptor_params,
                             const std::string& dbow_file_path,
                             const std::string& timestamps_file_path) {
  // construct detector and descriptor
  detector_ = beam_cv::GFTTDetector(detector_params);
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
  std::ofstream timestamps_file(imestamps_file_path);
  timestamps_file << std::setw(4) << index_to_timestamp_map_ << std::endl;
}

cv::Mat ImageDatabase::GetWord(const cv::Mat& descriptor) {
  auto vocab = bow_db_->getVocabulary();
  auto word_id = vocab.transform(descriptor);
  return vocab.getWord(word_id);
}

std::vector<uint64_t> ImageDatabase::QueryDatabase(const cv::Mat& query_image,
                                                   int N = 2) {
  std::vector<cv::KeyPoint> kps = detector_.DetectFeatures(query_image);
  cv::Mat features = descriptor_.ExtractDescriptors(query_image, kps);
  DBoW3::QueryResults results;
  bow_db_->query(features, results, N);

  std::vector<uint> indices;
  for (unsigned int i = 0; i < results.size(); i++) {
    DBoW3::Result res = results[i];
    indices.push_back(res.Id);
  }
  return indices;
}

uint64_t ImageDatabase::AddImage(const cv::Mat& image,
                                 const ros::Time& timestamp) {
  std::vector<cv::KeyPoint> kps = detector_.DetectFeatures(image);
  cv::Mat features = descriptor_.ExtractDescriptors(image, kps);
  unsigned int idx = bow_db_->add(features);
  index_to_timestamp_map_[idx] = timestamp.toSec();
  return idx;
}

beam::opt<ros::Time> ImageDatabase::GetImageTimestamp(uint64_t index) {
  if (!index_to_timestamp_map_.contains(index)) { return {}; }
  double time = index_to_timestamp_map_[index];
  return ros::Time(time);
}

} // namespace beam_cv
