#include <beam_cv/descriptors/BRISKDescriptor.h>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

namespace beam_cv {

void BRISKDescriptor::Params::LoadFromJson(const std::string& config_path) {
  if (config_path.empty()) { return; }

  if (!boost::filesystem::exists(config_path)) {
    BEAM_ERROR("Invalid file path for BRISK descriptor params, using default "
               "params. Input: {}",
               config_path);
    return;
  }

  nlohmann::json J;
  std::ifstream file(config_path);
  file >> J;

  rlist.clear();
  for (const auto& value : J["rlist"]) { rlist.push_back(value.get<float>()); }

  nlist.clear();
  for (const auto& value : J["nlist"]) { nlist.push_back(value.get<int>()); }

  d_max = J["d_max"];
  d_min = J["d_min"];
}

BRISKDescriptor::BRISKDescriptor(const Params& params) : params_(params){Setup();};

// Default constructor. Struct may be default or user defined.
BRISKDescriptor::BRISKDescriptor(const std::vector<float>& rlist,
                                 const std::vector<int>& nlist, float d_max,
                                 float d_min) {
  params_.rlist = rlist;
  params_.nlist = nlist;
  params_.d_max = d_max;
  params_.d_min = d_min;
  Setup();
}

void BRISKDescriptor::Setup(){
  // Ensure parameters are valid
  CheckConfig();

  // OpenCV refers to this as a parameter for "index remapping of the bits."
  // Kaehler and Bradski's book, "Learning OpenCV3: Computer Vision in C++
  // with the OpenCV Library" states this parameter is unused, and should be
  // omitted.
  std::vector<int> index_change;

  // Create cv::BRISK object with the desired parameters
  brisk_descriptor_ = cv::BRISK::create(
      params_.rlist, params_.nlist, params_.d_max, params_.d_min, index_change);
}

void BRISKDescriptor::CheckConfig() {
  // Check that the size of radiusList and numberList are equal and positive
  if (params_.rlist.size() == 0) {
    throw std::invalid_argument("No parameters in rlist!");
  } else if (params_.nlist.size() == 0) {
    throw std::invalid_argument("No parameters in nlist!");
  } else if (params_.rlist.size() != params_.nlist.size()) {
    throw std::invalid_argument("rlist and nlist are of unequal size!");
  }

  // Ensure all values of radiusList are positive
  for (const auto& radius : params_.rlist) {
    if (radius < 0) {
      throw std::invalid_argument("radius_list has a negative parameter!");
    }
  }

  // Ensure all values of numberList are positive
  for (auto& num_points : params_.nlist) {
    if (num_points < 0) {
      throw std::invalid_argument("nlist has a negative parameter!");
    }
  }

  // Ensure dMax and dMin are both positive, and check dMax is less than dMin
  if (params_.d_max < 0) {
    throw std::invalid_argument("d_max is a negative value!");
  } else if (params_.d_min < 0) {
    throw std::invalid_argument("d_min is a negative value!");
  } else if (params_.d_max > params_.d_min) {
    throw std::invalid_argument("d_max is greater than d_min!");
  }
}

cv::Mat
    BRISKDescriptor::ExtractDescriptors(const cv::Mat& image,
                                        std::vector<cv::KeyPoint>& keypoints) {
  cv::Mat descriptors;
  brisk_descriptor_->compute(image, keypoints, descriptors);
  return descriptors;
}
} // namespace beam_cv
