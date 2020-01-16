#include "beam_cv/DepthSuperpixels.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ximgproc.hpp>

using namespace std;

namespace beam_cv {

DepthSuperpixels::DepthSuperpixels(cv::Mat rgb_image, cv::Mat1f depth_image,
                                   bool write) {
  depth_image_ = std::make_shared<cv::Mat1f>(depth_image);
  rgb_image_ = std::make_shared<cv::Mat>(rgb_image);
  this->PerformSegmentation(10, 40, 10, write);
}

void DepthSuperpixels::PerformSegmentation(int region_size, float smoothness,
                                           int iterations, bool write) {
  cv::Ptr<cv::ximgproc::SuperpixelSLIC> slic =
      cv::ximgproc::createSuperpixelSLIC(*rgb_image_, cv::ximgproc::SLICO,
                                         region_size, smoothness);
  slic->iterate(iterations);
  slic->getLabels(labels_);
  num_superpixels = slic->getNumberOfSuperpixels();
  for (int i = 0; i < labels_.rows; i++) {
    for (int j = 0; j < labels_.cols; j++) {
      int key = labels_.at<int>(i, j);
      cv::Point2i p(i, j);
      if (superpixels_.find(key) == superpixels_.end()) {
        std::shared_ptr<SuperPixel> sp = std::make_shared<SuperPixel>();
        sp->pixels.push_back(p);
        superpixels_[key] = sp;
      } else {
        superpixels_[key]->pixels.push_back(p);
      }
    }
  }
  this->ExtractSuperpixelCentroids();
  this->FillSuperpixelDepth();
  // this->ExtractNeighbours();

  if (write) {
    cv::Mat result = rgb_image_->clone();
    cv::Mat mask;
    slic->getLabelContourMask(mask, true);
    result.setTo(cv::Scalar(0, 0, 255), mask);
    imwrite("/home/jake/result.png", result);
  }
}

std::unordered_map<int, std::shared_ptr<SuperPixel>>
    DepthSuperpixels::GetSuperpixels() {
  return superpixels_;
}

void DepthSuperpixels::ExtractSuperpixelCentroids() {
  for (int i = 0; i < num_superpixels; i++) {
    std::shared_ptr<SuperPixel> sp = superpixels_[i];
    int cx = 0;
    int cy = 0;
    for (cv::Point2i p : sp->pixels) {
      cx += p.x;
      cy += p.y;
    }
    cx /= sp->pixels.size();
    cy /= sp->pixels.size();
    cv::Point2i cent(cx, cy);
    sp->centroid = cent;
  }
}

void DepthSuperpixels::FillSuperpixelDepth() {
  for (int i = 0; i < num_superpixels; i++) {
    std::shared_ptr<SuperPixel> sp = superpixels_[i];
    std::vector<cv::Point2i> pixels;
    std::vector<float> depths;
    int num_obs = 0;
    for (cv::Point2i p : sp->pixels) {
      float d = depth_image_->at<float>(p.x, p.y);
      if (d > 0) {
        num_obs++;
        pixels.push_back(p);
        depths.push_back(d);
      }
    }
    Eigen::MatrixXf X(num_obs, 2);
    Eigen::RowVectorXf y(num_obs);
    for (int i = 0; i < num_obs; i++) {
      X(i, 0) = pixels[i].x;
      X(i, 1) = pixels[i].y;
      y(i) = depths[i];
    }
    sp->depth_values = std::make_tuple(X, y);
  }
}

void DepthSuperpixels::ExtractNeighbours() {
  for (int key = 0; key < num_superpixels; key++) {
    std::shared_ptr<SuperPixel> sp = superpixels_[key];
    for (cv::Point2i p : sp->pixels) {
      cv::Point2i n;
      for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
          n.x = p.x + i;
          n.y = p.y + j;
          int neighbour_label = labels_.at<int>(n.x, n.y);
          if (neighbour_label != key) {
            sp->neighbours.insert(neighbour_label);
          }
        }
      }
    }
  }
}

} // namespace beam_cv