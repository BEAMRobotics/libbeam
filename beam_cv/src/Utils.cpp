// beam
#include "beam_cv/Utils.h"
#include "beam_utils/math.hpp"
#include "beam_utils/uf.hpp"
// std
#include <algorithm>

namespace beam_cv {

cv::Mat VisualizeDepthImage(const cv::Mat& input) {
  cv::Mat image = input.clone();
  float max_depth = 0;
  image.forEach<float>([&](float& distance, const int* position) -> void {
    (void)position;
    if (distance > max_depth) { max_depth = distance; }
  });
  cv::Mat gs_depth = cv::Mat(image.rows, image.cols, CV_8UC1);
  image.forEach<float>([&](float& distance, const int* position) -> void {
    int scale = 255 / max_depth;
    uint8_t pixel_value = (scale * distance);
    gs_depth.at<uchar>(position[0], position[1]) = pixel_value;
  });
  applyColorMap(gs_depth, gs_depth, cv::COLORMAP_JET);
  return gs_depth;
}

cv::Mat AdaptiveHistogram(const cv::Mat& input) {
  cv::Mat lab_image;
  cv::cvtColor(input, lab_image, CV_BGR2Lab);
  std::vector<cv::Mat> lab_planes(6);
  cv::split(lab_image, lab_planes);
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  clahe->setClipLimit(3);
  cv::Mat dst;
  clahe->apply(lab_planes[0], dst);
  dst.copyTo(lab_planes[0]);
  cv::merge(lab_planes, lab_image);
  cv::Mat new_image;
  cv::cvtColor(lab_image, new_image, CV_Lab2BGR);
  return new_image;
}

cv::Mat KMeans(const cv::Mat& input, int K) {
  cv::Mat image = input.clone();
  cv::Size og(image.cols, image.rows);
  cv::Size half(image.cols / 2, image.rows / 2);
  cv::resize(image, image, half);
  cv::Mat data;
  image.convertTo(data, CV_32F);
  data = data.reshape(1, data.total());
  // do kmeans
  cv::Mat labels, centers;
  cv::kmeans(data, K, labels, cv::TermCriteria(CV_TERMCRIT_ITER, 10, 1.0), 3,
             cv::KMEANS_PP_CENTERS, centers);
  // reshape both to a single row of Vec3f pixels:
  centers = centers.reshape(3, centers.rows);
  data = data.reshape(3, data.rows);
  // replace pixel values with their center value:
  cv::Vec3f* p = data.ptr<cv::Vec3f>();
  for (int i = 0; i < (int)data.rows; i++) {
    int center_id = labels.at<int>(i);
    p[i] = centers.at<cv::Vec3f>(center_id);
  }
  // back to 2d, and uchar:
  image = data.reshape(3, image.rows);
  image.convertTo(image, CV_8UC1);
  cv::Mat grey;
  cv::cvtColor(image, grey, CV_BGR2GRAY);
  cv::morphologyEx(grey, grey, cv::MORPH_CLOSE, beam::GetFullKernel(5));
  cv::resize(grey, grey, og);
  return grey;
}

cv::Mat ExtractSkeleton(const cv::Mat& input_image) {
  cv::Mat output_image = input_image.clone();
  cv::Mat skel(output_image.size(), CV_8UC1, cv::Scalar(0));
  cv::Mat temp(output_image.size(), CV_8UC1);
  // Declare structuring element for open function
  cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
  bool done = false;
  while (!done) {
    cv::morphologyEx(output_image, temp, cv::MORPH_OPEN, element);
    cv::bitwise_not(temp, temp);
    cv::bitwise_and(output_image, temp, temp);
    cv::bitwise_or(skel, temp, skel);
    cv::erode(output_image, output_image, element);

    double max;
    cv::minMaxLoc(output_image, 0, &max);
    done = (max == 0);
  }
  return skel;
}

cv::Mat RemoveClusters(const cv::Mat& input_image, int threshold) {
  // Remove small pixel clusters
  cv::Mat output_image = input_image.clone();
  cv::Mat category(output_image.size(), CV_16UC1);
  cv::Mat stats, my_centroids;
  int connectivity = 8;
  int itype = CV_16U;
  int num_comp = cv::connectedComponentsWithStats(
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

std::vector<cv::Mat> SegmentComponents(const cv::Mat& image) {
  // Create a copy of the binary image
  cv::Mat im_category(image.size(), CV_16UC1);
  cv::Mat my_stats, my_centroids;
  int connectivity = 8;
  int itype = CV_16U;
  int num_comp = cv::connectedComponentsWithStats(
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

std::map<int, std::vector<cv::Point2i>>
    ConnectedComponents(const cv::Mat& image) {
  auto UnionCoords = [](cv::Mat image, int x, int y, int x2, int y2,
                        beam::UF& uf) {
    if (y2 < image.cols && x2 < image.rows &&
        image.at<uchar>(x, y) == image.at<uchar>(x2, y2)) {
      uf.UnionSets(x * image.cols + y, x2 * image.cols + y2);
    }
  };
  beam::UF uf;
  uf.Initialize(image.rows * image.cols);
  for (int x = 0; x < image.rows; x++) {
    for (int y = 0; y < image.cols; y++) {
      UnionCoords(image, x, y, x + 1, y, uf);
      UnionCoords(image, x, y, x, y + 1, uf);
    }
  }
  std::map<int, std::vector<cv::Point2i>> sets;
  for (int i = 0; i < image.rows * image.cols; i++) {
    int seti = uf.FindSet(i);
    if (sets.count(seti) != 0) {
      cv::Point2i p(i / image.cols, i % image.cols);
      sets[seti].push_back(p);
    } else {
      std::vector<cv::Point2i> points;
      sets.insert({seti, points});
    }
  }
  return sets;
}

double PixelDistance(cv::Point2i p1, cv::Point2i p2) {
  double distance =
      sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
  return distance;
}

Eigen::Vector2i FindClosest(Eigen::Vector2i search_pixel, cv::Mat depth_image) {
  cv::Point2i sp(search_pixel[0], search_pixel[1]);
  std::vector<double> distances;
  std::vector<cv::Point2i> pixels;
  depth_image.forEach<uchar>([&](uchar& pixel, const int* position) -> void {
    if (pixel > 0) {
      cv::Point2i p(position[0], position[1]);
      double d = PixelDistance(sp, p);
      distances.push_back(d);
      pixels.push_back(p);
    }
  });
  int min_index =
      std::min_element(distances.begin(), distances.end()) - distances.begin();
  Eigen::Vector2i output(pixels[min_index].x, pixels[min_index].y);
  return output;
}
} // namespace beam_cv
