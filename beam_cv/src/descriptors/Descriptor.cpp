#include <beam_cv/descriptors/Descriptors.h>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

#include <beam_utils/log.h>

namespace beam_cv {

std::shared_ptr<Descriptor> Descriptor::Create(DescriptorType type,
                                               const std::string& file_path) {
  // check input file path
  std::string file_to_read = file_path;
  if (!boost::filesystem::exists(file_to_read) && !file_path.empty()) {
    BEAM_WARN(
        "Invalid file path for descriptor, using default params. Input: {}",
        file_path);
    file_to_read = "";
  }

  // create detector
  if (type == DescriptorType::ORB) {
    ORBDescriptor::Params params;
    params.LoadFromJson(file_to_read);
    return std::make_shared<ORBDescriptor>(params);
  } else if (type == DescriptorType::SIFT) {
    SIFTDescriptor::Params params;
    params.LoadFromJson(file_to_read);
    return std::make_shared<SIFTDescriptor>(params);
  } else if (type == DescriptorType::BRISK) {
    BRISKDescriptor::Params params;
    params.LoadFromJson(file_to_read);
    return std::make_shared<BRISKDescriptor>(params);
  } else if (type == DescriptorType::BEBLID) {
    BEBLIDDescriptor::Params params;
    params.LoadFromJson(file_to_read);
    return std::make_shared<BEBLIDDescriptor>(params);
  } else {
    BEAM_WARN("Input descriptor type not implemented. Using default.");
    ORBDescriptor::Params params;
    params.LoadFromJson(file_to_read);
    return std::make_shared<ORBDescriptor>(params);
  }
}

} // namespace beam_cv
