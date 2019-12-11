/** @file
 * @ingroup cv
 */

#pragma once
// beam
#include "beam_utils/math.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv/cv.h>

namespace beam_cv {

struct SuperPixel {
  std::vector<cv::Point2i> pixels;
  std::set<int> neighbours;
  std::tuple<Eigen::MatrixXd, Eigen::RowVectorXd> depth_values;
  cv::Point2i centroid;
};

class DepthSuperpixels {
public:
  /**
   * @brief Default constructor
   */
  DepthSuperpixels() = default;

  /**
   * @brief Constructor to initliaze image variables
   * @param rgb_image the color image
   * @param depth_image the float balued image representing depth
   */
  DepthSuperpixels(cv::Mat rgb_image, cv::Mat1f depth_image);

  /**
   * @brief Default destructor
   */
  ~DepthSuperpixels() = default;

  /**
   * @brief Performs superpixel segmentation and fills superpixels_ variable
   */
  void PerformSegmentation(int region_size, float smoothness, int iterations,
                           bool write);

  /**
   * @brief Performs superpixel segmentation and fills superpixels_ variable
   */
  std::unordered_map<int, std::shared_ptr<SuperPixel>> GetSuperpixels();

private:
  /**
   * @brief Performs superpixel segmentation and fills superpixels_ variable
   */
  void ExtractSuperpixelCentroids();

  /**
   * @brief Performs superpixel segmentation and fills superpixels_ variable
   */
  void FillSuperpixelDepth();

  /**
   * @brief Performs superpixel segmentation and fills superpixels_ variable
   */
  void ExtractNeighbours();

protected:
  int num_superpixels;
  std::shared_ptr<cv::Mat> rgb_image_;
  cv::Mat labels_;
  std::shared_ptr<cv::Mat1f> depth_image_;
  std::unordered_map<int, std::shared_ptr<SuperPixel>> superpixels_;
};

} // namespace beam_cv