#include <beam_cv/OpenCVConversions.h>

#include <regex>

#include <boost/endian/conversion.hpp>
#include <sensor_msgs/image_encodings.h>

namespace enc = sensor_msgs::image_encodings;

namespace beam_cv {

// ImgToMat helper
int OpenCVConversions::DepthStrToInt(const std::string depth) {
  if (depth == "8U") {
    return 0;
  } else if (depth == "8S") {
    return 1;
  } else if (depth == "16U") {
    return 2;
  } else if (depth == "16S") {
    return 3;
  } else if (depth == "32S") {
    return 4;
  } else if (depth == "32F") {
    return 5;
  }
  return 6;
}

// ImgToMat helper
int OpenCVConversions::GetCvType(const std::string& encoding) {
  // Check for the most common encodings first
  if (encoding == enc::BGR8) return CV_8UC3;
  if (encoding == enc::MONO8) return CV_8UC1;
  if (encoding == enc::RGB8) return CV_8UC3;
  if (encoding == enc::MONO16) return CV_16UC1;
  if (encoding == enc::BGR16) return CV_16UC3;
  if (encoding == enc::RGB16) return CV_16UC3;
  if (encoding == enc::BGRA8) return CV_8UC4;
  if (encoding == enc::RGBA8) return CV_8UC4;
  if (encoding == enc::BGRA16) return CV_16UC4;
  if (encoding == enc::RGBA16) return CV_16UC4;

  // For bayer, return one-channel
  if (encoding == enc::BAYER_RGGB8) return CV_8UC1;
  if (encoding == enc::BAYER_BGGR8) return CV_8UC1;
  if (encoding == enc::BAYER_GBRG8) return CV_8UC1;
  if (encoding == enc::BAYER_GRBG8) return CV_8UC1;
  if (encoding == enc::BAYER_RGGB16) return CV_16UC1;
  if (encoding == enc::BAYER_BGGR16) return CV_16UC1;
  if (encoding == enc::BAYER_GBRG16) return CV_16UC1;
  if (encoding == enc::BAYER_GRBG16) return CV_16UC1;

  // Miscellaneous
  if (encoding == enc::YUV422) return CV_8UC2;

  // Check all the generic content encodings
  std::cmatch m;

  if (std::regex_match(
          encoding.c_str(), m,
          std::regex("(8U|8S|16U|16S|32S|32F|64F)C([0-9]+)"))) {
    return CV_MAKETYPE(DepthStrToInt(m[1].str()), atoi(m[2].str().c_str()));
  }

  if (std::regex_match(encoding.c_str(), m,
                         std::regex("(8U|8S|16U|16S|32S|32F|64F)"))) {
    return CV_MAKETYPE(DepthStrToInt(m[1].str()), 1);
  }

  throw std::runtime_error("Unrecognized image encoding [" + encoding + "]");
}

cv::Mat OpenCVConversions::ImgToMat(const sensor_msgs::Image& source) {
  int source_type = GetCvType(source.encoding);
  int byte_depth = enc::bitDepth(source.encoding) / 8;
  int num_channels = enc::numChannels(source.encoding);

  if (source.step < source.width * byte_depth * num_channels) {
    std::stringstream ss;
    ss << "Image is wrongly formed: step < width * byte_depth * num_channels  "
          "or  "
       << source.step << " != " << source.width << " * " << byte_depth << " * "
       << num_channels;
    throw std::runtime_error(ss.str());
  }

  if (source.height * source.step != source.data.size()) {
    std::stringstream ss;
    ss << "Image is wrongly formed: height * step != size  or  "
       << source.height << " * " << source.step << " != " << source.data.size();
    throw std::runtime_error(ss.str());
  }

  // If the endianness is the same as locally, share the data
  cv::Mat mat(source.height, source.width, source_type,
              const_cast<uchar*>(&source.data[0]), source.step);
  if ((boost::endian::order::native == boost::endian::order::big &&
       source.is_bigendian) ||
      (boost::endian::order::native == boost::endian::order::little &&
       !source.is_bigendian) ||
      byte_depth == 1)
    return mat;

  // Otherwise, reinterpret the data as bytes and switch the channels
  // accordingly
  mat = cv::Mat(source.height, source.width,
                CV_MAKETYPE(CV_8U, num_channels * byte_depth),
                const_cast<uchar*>(&source.data[0]), source.step);
  cv::Mat mat_swap(source.height, source.width, mat.type());

  std::vector<int> fromTo;
  fromTo.reserve(num_channels * byte_depth);
  for (int i = 0; i < num_channels; ++i)
    for (int j = 0; j < byte_depth; ++j) {
      fromTo.push_back(byte_depth * i + j);
      fromTo.push_back(byte_depth * i + byte_depth - 1 - j);
    }
  cv::mixChannels(std::vector<cv::Mat>(1, mat),
                  std::vector<cv::Mat>(1, mat_swap), fromTo);

  // Interpret mat_swap back as the proper type
  mat_swap.reshape(num_channels);

  return mat_swap;
}
} // namespace beam_cv