#include <beam_cv/matchers/Matchers.h>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

namespace beam_cv {

std::shared_ptr<Matcher> Matcher::Create(MatcherType type,
                                         const std::string& file_path) {
  // check input file path
  std::string file_to_read = file_path;
  if (!boost::filesystem::exists(file_to_read) && !file_path.empty()) {
    BEAM_WARN("Invalid file path for detector, using default params. Input: {}",
              file_path);
    file_to_read = "";
  }

  // create matcher
  if (type == MatcherType::BF) {
    BFMatcher::Params params;
    params.LoadFromJson(file_to_read);
    return std::make_shared<BFMatcher>(params);
  } else if (type == MatcherType::FLANN) {
    FLANNMatcher::Params params;
    params.LoadFromJson(file_to_read);
    return std::make_shared<FLANNMatcher>(params);
  } else {
    BEAM_WARN("Input matcher type not implemented. Using default.");
    BFMatcher::Params params;
    params.LoadFromJson(file_to_read);
    return std::make_shared<BFMatcher>(params);
  }
}

}; // namespace beam_cv