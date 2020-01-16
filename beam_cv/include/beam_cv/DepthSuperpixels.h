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
  std::tuple<Eigen::MatrixXf, Eigen::RowVectorXf> depth_values;
  cv::Point2i centroid;
};

class DepthSuperpixels {
public:
  int num_superpixels;
  /**
   * @brief Default constructor
   */
  DepthSuperpixels() = default;

  /**
   * @brief Constructor to initliaze image variables
   * @param rgb_image the color image
   * @param depth_image the float balued image representing depth
   */
  DepthSuperpixels(cv::Mat rgb_image, cv::Mat1f depth_image, bool write);

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
   * @brief Returns map of superpixels
   */
  std::unordered_map<int, std::shared_ptr<SuperPixel>> GetSuperpixels();

private:
  /**
   * @brief Finds centorid of each superpixel
   */
  void ExtractSuperpixelCentroids();

  /**
   * @brief Fills each superpixel with its associated epth info
   */
  void FillSuperpixelDepth();

  /**
   * @brief finds neighbours of each superpixel
   */
  void ExtractNeighbours();

protected:
  std::shared_ptr<cv::Mat> rgb_image_;
  cv::Mat labels_;
  std::shared_ptr<cv::Mat1f> depth_image_;
  std::unordered_map<int, std::shared_ptr<SuperPixel>> superpixels_;
};

} // namespace beam_cv