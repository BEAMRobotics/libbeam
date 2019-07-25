#include "beam_cv/Morphology.h"
#include "beam_utils/math.hpp"
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

std::vector<Mat> SegmentSkeleton(const Mat& skeleton, const Mat& image) {
  // Create a copy of the binary image
  Mat image_copy = image.clone();
  Mat im_category(image.size(), CV_16UC1);
  Mat my_stats, my_centroids;
  int connectivity = 8;
  int itype = CV_16U;
  int num_comp = connectedComponentsWithStats(
      image, im_category, my_stats, my_centroids, connectivity, itype);
  im_category.convertTo(im_category, CV_8UC1);
  // Calc average width (pixels) by dividing total area by total length
  double avg_width = sum(image_copy)[0] / sum(skeleton)[0];
  // Example forced width calculation
  int resolution = std::nearbyint(std::sqrt(avg_width) * 0.5f) * 2.0f * 5 + 1;
  // int resolution = 9;
  int stepper = (resolution - 1) / 2;
  double scale = std::floor(1.0 / num_comp * 255); // scale for colormap
  // initialize smaller window matrices
  Mat im_orig(resolution, resolution, CV_8UC1);
  Mat im_skel(resolution, resolution, CV_8UC1);
  Mat cm_skel(image.size(), CV_8UC1, Scalar(0));
  std::set<int> crack_vals;
  for (int x = 0; x < skeleton.rows; ++x) // iterate over skeleton image
  {
    for (int y = 0; y < skeleton.cols; ++y) // iterate over skeleton image
    {
      int crack_num = int(im_category.at<uchar>(x, y));
      int val = crack_num * scale;
      cm_skel.at<uchar>(x, y) = val;
      if (crack_vals.find(val) == crack_vals.end()) { crack_vals.insert(val); }
      if (skeleton.at<uchar>(x, y) == 255) // if skeleton pixel
      {
        for (int i = 0; i < im_skel.rows; ++i) // iterate over window
        {
          for (int j = 0; j < im_skel.cols; ++j) {
            // populate window with the values from skeleton and binary image
            int idx_1 = x + i - stepper;
            int idx_2 = y + j - stepper;

            im_orig.at<uchar>(i, j) = image_copy.at<uchar>(idx_1, idx_2);
            im_skel.at<uchar>(i, j) = skeleton.at<uchar>(idx_1, idx_2);
          }
        }
      }
    }
  }
  std::vector<Mat> cracks;
  for (int v : crack_vals) {
    cv::Mat crack(image.size(), CV_8UC1);
    for (int row = 0; row < cm_skel.rows; row++) {
      for (int col = 0; col < cm_skel.cols; col++) {
        int val = cm_skel.at<uchar>(row, col);
        if (val == v) { crack.at<uchar>(row, col) = 255; }
      }
    }
    cracks.push_back(crack);
  }
  return cracks;
}
} // namespace beam_cv