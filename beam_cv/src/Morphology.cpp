// beam
#include "beam_cv/Morphology.h"
#include "beam_utils/math.hpp"
// std
#include <algorithm>

using namespace cv;

namespace beam_cv {

Mat ExtractSkeleton(const Mat& input_image) {
  Mat output_image = input_image.clone();
  Mat skel(output_image.size(), CV_8UC1, Scalar(0));
  Mat temp(output_image.size(), CV_8UC1);
  // Declare structuring element for open function
  Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));

  bool done = false;
  while (!done) {
    morphologyEx(output_image, temp, MORPH_OPEN, element);
    bitwise_not(temp, temp);
    bitwise_and(output_image, temp, temp);
    bitwise_or(skel, temp, skel);
    erode(output_image, output_image, element);

    double max;
    minMaxLoc(output_image, 0, &max);
    done = (max == 0);
  }
  return skel;
}

Mat RemoveClusters(const Mat& input_image, int threshold) {
  // Remove small pixel clusters
  Mat output_image = input_image.clone();
  Mat category(output_image.size(), CV_16UC1);
  Mat stats, my_centroids;
  int connectivity = 8;
  int itype = CV_16U;
  int num_comp = connectedComponentsWithStats(
      output_image, category, stats, my_centroids, connectivity, itype);
  BEAM_INFO("Number of Components in image: {}", num_comp);

  category.convertTo(category, CV_8UC1);
  for (int x = 0; x < output_image.rows; ++x) // iterate over skeleton image
  {
    for (int y = 0; y < output_image.cols; ++y) // iterate over skeleton image
    {
      int seg_category = int(category.at<uchar>(x, y));
      if (seg_category > 0) {
        if (stats.at<int>(seg_category, 4) < threshold) {
          output_image.at<uchar>(x, y) = 0;
        }
      }
    }
  }
  return output_image;
}

std::vector<Mat> SegmentComponents(const Mat& image) {
  // Create a copy of the binary image
  Mat im_category(image.size(), CV_16UC1);
  Mat my_stats, my_centroids;
  int connectivity = 8;
  int itype = CV_16U;
  int num_comp = connectedComponentsWithStats(
      image, im_category, my_stats, my_centroids, connectivity, itype);
  im_category.convertTo(im_category, CV_8UC1);
  // initialize smaller window matrices
  std::vector<cv::Mat> cracks;
  for (int i = 0; i < num_comp; i++) {
    cv::Mat crack(image.size(), CV_8UC1);
    cracks.push_back(crack);
  }
  for (int x = 0; x < image.rows; ++x) // iterate over skeleton image
  {
    for (int y = 0; y < image.cols; ++y) // iterate over skeleton image
    {
      int crack_num = int(im_category.at<uchar>(x, y));
      cracks[crack_num].at<uchar>(x, y) = 255;
    }
  }
  cracks.erase(cracks.begin());
  return cracks;
}

Mat ConnectSkeleton(const Mat& image) {}
} // namespace beam_cv