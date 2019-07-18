#include <cmath>
#include <iostream>
#include <map>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "beam_cv/Morphology.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
  if (argc != 3) {
    cout << " Usage: imskeleton ImageToLoadAndDisplay BinaryThreshold" << endl;
    return -1;
  }
  beam_cv::Morphology morph;
  Mat image = imread(argv[1], 0); // Read the file
  if (!image.data)                // Check for invalid input
  {
    cout << "Could not open or find the image" << std::endl;
    return -1;
  }

  // Get theshold of input
  int thresh;
  std::stringstream ss(argv[2]);
  ss >> thresh;

  // process image used (just temp with example image)
  cv::threshold(image, image, thresh, 255, cv::THRESH_BINARY);
  // Initialize skel image and temp storage

  image = morph.RemoveClusters(image, 100);
  image = morph.CloseObjects(image);
  cv::Mat skeleton = morph.ExtractSkeleton(image);

  // Create a copy of the binary image
  cv::Mat image_copy = image.clone();

  cv::Mat im_category(image.size(), CV_16UC1);
  cv::Mat my_stats, my_centroids;
  int connectivity = 8;
  int itype = CV_16U;
  int num_comp = cv::connectedComponentsWithStats(
      image, im_category, my_stats, my_centroids, connectivity, itype);
  im_category.convertTo(im_category, CV_8UC1);

  // Initialize map for widths
  std::map<int, std::vector<double>> crack_map;

  // Calc the length of the crack
  int crack_length = cv::sum(skeleton)[0] / 255;
  std::cout << "Total Length : " << crack_length << std::endl;

  // Calc average width (pixels) by dividing total area by total length
  double avg_width = cv::sum(image_copy)[0] / cv::sum(skeleton)[0];
  std::cout << "Average width: " << avg_width << " pixels." << std::endl;

  // Example forced width calculation
  int resolution = std::nearbyint(std::sqrt(avg_width) * 0.5f) * 2.0f * 5 + 1;
  // int resolution = 9;
  std::cout << "Filter Resolution: " << resolution << std::endl;
  int stepper = (resolution - 1) / 2;
  double scale = std::floor(1.0 / num_comp * 255); // scale for colormap

  // initialize smaller window matrices
  cv::Mat im_orig(resolution, resolution, CV_8UC1);
  cv::Mat im_skel(resolution, resolution, CV_8UC1);
  cv::Mat cm_skel(image.size(), CV_8UC1, cv::Scalar(0));
  for (int x = 0; x < skeleton.rows; ++x) // iterate over skeleton image
  {
    for (int y = 0; y < skeleton.cols; ++y) // iterate over skeleton image
    {
      int crack_num = int(im_category.at<uchar>(x, y));
      cm_skel.at<uchar>(x, y) = crack_num * scale;
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

        double width = cv::sum(im_orig)[0] / cv::sum(im_skel)[0];
        crack_map[crack_num].push_back(width);
      }
    }
  }

  for (int i = 1; i < num_comp; ++i) // 0 is all points not showing crack
  {
    std::cout << "Results for Crack: " << i << std::endl;
    if (crack_map[i].size() > 50) {
      std::cout << "Crack Length: " << crack_map[i].size() << std::endl;
      double max_width = *max_element(crack_map[i].begin(), crack_map[i].end());
      double min_width = *min_element(crack_map[i].begin(), crack_map[i].end());
      std::cout << "Max: " << max_width << " and Min: " << min_width
                << std::endl;
    } else {
      std::cout << "Not enough data to yield meaningful results" << std::endl;
    };
  }

  applyColorMap(cm_skel, cm_skel, COLORMAP_HOT);

  namedWindow("Original Image", WINDOW_NORMAL);
  namedWindow("Image Skeleton", WINDOW_NORMAL);
  imshow("Original Image", cm_skel);
  imshow("Image Skeleton", skeleton);
  waitKey(0); // Wait for a keystroke in the window
  return 0;
}