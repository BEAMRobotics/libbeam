// beam
#include "beam_cv/Utils.h"
#include "beam_utils/math.hpp"
// std
#include <algorithm>

using namespace cv;

namespace beam_cv {

Mat VisualizeDepthImage(const Mat& input) {
  Mat image = input.clone();
  float max_depth = 0;
  image.forEach<float>([&](float& distance, const int* position) -> void {
    if (distance > max_depth) { max_depth = distance; }
  });
  Mat gs_depth = Mat(image.rows, image.cols, CV_8UC1);
  image.forEach<float>([&](float& distance, const int* position) -> void {
    int scale = 255 / max_depth;
    uint8_t pixel_value = (scale * distance);
    gs_depth.at<uchar>(position[0], position[1]) = pixel_value;
  });
  applyColorMap(gs_depth, gs_depth, COLORMAP_JET);
  return gs_depth;
}

Mat AdaptiveHistogram(const Mat& input) {
  Mat lab_image;
  cvtColor(input, lab_image, CV_BGR2Lab);
  std::vector<Mat> lab_planes(6);
  split(lab_image, lab_planes);
  Ptr<CLAHE> clahe = createCLAHE();
  clahe->setClipLimit(3);
  Mat dst;
  clahe->apply(lab_planes[0], dst);
  dst.copyTo(lab_planes[0]);
  merge(lab_planes, lab_image);
  Mat new_image;
  cvtColor(lab_image, new_image, CV_Lab2BGR);
  return new_image;
}

Mat KMeans(const Mat& input, int K) {
  BEAM_INFO("Performing K Means Segmentation...");
  Mat image = input.clone();
  Mat data;
  image.convertTo(data, CV_32F);
  data = data.reshape(1, data.total());
  // do kmeans
  Mat labels, centers;
  kmeans(data, K, labels, TermCriteria(CV_TERMCRIT_ITER, 10, 1.0), 3,
         KMEANS_PP_CENTERS, centers);
  // reshape both to a single row of Vec3f pixels:
  centers = centers.reshape(3, centers.rows);
  data = data.reshape(3, data.rows);
  // replace pixel values with their center value:
  Vec3f* p = data.ptr<Vec3f>();
  for (size_t i = 0; i < data.rows; i++) {
    int center_id = labels.at<int>(i);
    p[i] = centers.at<Vec3f>(center_id);
  }
  // back to 2d, and uchar:
  image = data.reshape(3, image.rows);
  image.convertTo(image, CV_8UC1);
  Mat grey;
  cvtColor(image, grey, CV_BGR2GRAY);
  BEAM_INFO("K Means Segmentation Complete");
  return grey;
}

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
  std::vector<Mat> cracks;
  for (int i = 0; i < num_comp; i++) {
    Mat crack(image.size(), CV_8UC1);
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

} // namespace beam_cv